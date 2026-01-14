import numpy as np
import argparse
import struct
import socket
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
import pinocchio as pin

#rclpy imports
import rclpy
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

#Imports needed to send things to the MPC controller
from agimus_msgs.msg import MpcInput
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from linear_feedback_controller_msgs.msg import Sensor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import copy

#Imports needed to get robot params
from agimus_controller.factory.robot_model import RobotModelParameters, RobotModels
from agimus_controller_ros.simple_trajectory_publisher import TrajectoryPublisherBase
from agimus_controller_ros.trajectory_weights_parameters import (
    trajectory_weights_params,
)
from agimus_controller_ros.ros_utils import (
    weighted_traj_point_to_mpc_msg,
    get_param_from_node,
)
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)

from agimus_demo_08_collision_avoidance.goal_publisher_parameters import (
    goal_publisher,
)

#Visualization imports
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA, Header, Int32
from rclpy.duration import Duration

@dataclass
class ControllerPose:
    matrix: np.ndarray  # shape (4,4)

@dataclass
class ControllerInput:
    joystick: tuple[float, float]
    index_trigger: float
    hand_trigger: float
    buttons: dict[str, bool]

@dataclass
class VRFrame:
    head_pose: ControllerPose
    left_pose: ControllerPose
    right_pose: ControllerPose
    left_input: ControllerInput
    right_input: ControllerInput


class QuestTrajectoryPublisher(TrajectoryPublisherBase):
    def __init__(self):
        super().__init__("quest_trajectory_publisher")
        #self.param_listener = goal_publisher.ParamListener(self)
        #self.params = self.param_listener.get_params()


        self.ee_frame_name = 'fer_hand'
        self.future_init_done = Future()
        self.future_trajectory_done = Future()
        self.get_logger().info("Simple trajectory publisher node started.")
        self._marker_base: Marker | None = None
        self._target_pose_marker_pub = self.create_publisher(Marker, "target_pose_marker", 10)
        
        #self.create_subscription(Sensor, "/linear_feedback_controller/sensor", self.sensor_cb, 10)
        #self.mpc_pub = self.create_publisher(MpcInput, "/linear_feedback_controller/mpc_input", 10)
        #self.publisher_.publish(mpc_msg)

        #I dont need this if Im not doing IK myself
        #pkg_dir = get_package_share_directory("franka_description")  # change to your package
        #urdf_path = os.path.join(pkg_dir, "robots", "common", "franka_robot.xacro") 
        #self.pin_model = pin.buildModelFromUrdf(urdf_path)
        #self.pin_data = pin.Data(self.pin_model)
        #self.q = None
        #self.dq = None

        #Set up quest stream
        UDP_IP = "0.0.0.0"
        UDP_PORT = 5000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)
        self.R_head_flat=None
        #self.last_pos=None
        #self.timer = None#self.create_timer(0.01, self.stream_quest_to_robot)
        self._marker_base = Marker(header=Header(frame_id="fer_link0"),
                ns="goal_publisher",
                type=Marker.SPHERE,
                action=Marker.ADD,
                scale=Vector3(x=0.05, y=0.05, z=0.05),
                color=ColorRGBA(
                    **dict(zip("rgba", [1.0, 0.0, 0.0, 0.5]))
                ),
                lifetime=Duration(seconds=0.01 * 10.0).to_msg(),
            )
        self.get_logger().info("Quest trajectory publisher started, waiting for Quest data...")

    def ready_callback(self):
        self.get_logger().info(
            f"QuestTrajectoryPublisher ready. q0 = {[round(v, 3) for v in self.q0]}"
        )

        # Start streaming only once everything is ready
        self.timer = self.create_timer(0.01, self.stream_quest_to_robot)
    #def sensor_cb(self, msg):
    #    self.q = np.array(msg.robot_configuration)
    #    self.get_logger().info("Robot configuration: "+str(self.q))
    #    self.dq = np.array(msg.robot_velocity)
    #    if not self.future_init_done.done():
    #        self.future_init_done.set_result(True)
    
    def decode_packet(self, data: bytes) -> VRFrame:
        floats = struct.unpack('<66f', data)
        f = iter(floats)

        def next_mat():
            return np.array([ [next(f), next(f), next(f), next(f)],
                            [next(f), next(f), next(f), next(f)],
                            [next(f), next(f), next(f), next(f)],
                            [next(f), next(f), next(f), next(f)] ], dtype=np.float32)

        head_pose  = ControllerPose(next_mat())
        left_pose  = ControllerPose(next_mat())
        right_pose = ControllerPose(next_mat())

        # Analog values
        left_joy   = (next(f), next(f))
        right_joy  = (next(f), next(f))
        left_index, right_index, left_grip, right_grip = next(f), next(f), next(f), next(f)

        # Buttons
        btnA, btnB, btnX, btnY, thumbL, thumbR, trigL, trigR, gripL, gripR = [int(next(f)) for _ in range(10)]

        left_buttons = {
            "X": bool(btnX),
            "Y": bool(btnY),
            "Thumbstick": bool(thumbL),
            "TriggerButton": bool(trigL),
            "GripButton": bool(gripL),
        }
        right_buttons = {
            "A": bool(btnA),
            "B": bool(btnB),
            "Thumbstick": bool(thumbR),
            "TriggerButton": bool(trigR),
            "GripButton": bool(gripR),
        }

        left_input  = ControllerInput(left_joy, left_index, left_grip, left_buttons)
        right_input = ControllerInput(right_joy, right_index, right_grip, right_buttons)

        return VRFrame(head_pose, left_pose, right_pose, left_input, right_input)

    def stream_quest_to_robot(self):
        #self.get_logger().info("Q zero: "+str(self.current_q))


        if self.current_q is None:
            return
        try:
            data, addr = self.sock.recvfrom(4096)
            frame = self.decode_packet(data)
            if not hasattr(self, "quest_ref"):
                #self.get_logger().info(" Robot models attributed: "+ str(dir(self.robot_models)))
                ee_id = self.robot_models.robot_model.getFrameId(self.ee_frame_name)
                data = pin.Data(self.robot_models.robot_model)
                pin.forwardKinematics(self.robot_models.robot_model, data, self.current_q)
                pin.updateFramePlacements(self.robot_models.robot_model, data)

                # Extract the end-effector SE3
                ee_pose = data.oMf[ee_id]
                self.quest_ref=ee_pose.homogeneous

                head_fwd=frame.head_pose.matrix[:3, 2]
                head_fwd[1]=0
                head_fwd=head_fwd/(np.linalg.norm(head_fwd)+1e-6)

                head_right=frame.head_pose.matrix[:3, 0]
                head_right[1]=0
                head_right=head_right/(np.linalg.norm(head_right)+1e-6)

                R_head_flat=np.stack((head_right, np.array([0, 1, 0]), head_fwd))
                self.R_head_flat=R_head_flat.copy()
                self.last_pos=frame.right_pose.matrix.copy()
                return
        except BlockingIOError:
            self.get_logger().info('No data from quest')
            return


        #self.get_logger().info("EE pos of the robot: "+str(frame.right_pose.matrix))
        #With this transformation matrix, the movements of the robot actually reflect the 
        R_rq = np.array([
                [ 0,  0,  1],   # Unity Z → Robot X
                [-1,  0,  0],   # Unity X → Robot -Y
                [ 0,  1,  0],   # Unity Y → Robot Z
            ])

        Tq=frame.right_pose.matrix
        #Tq0 = self.quest_ref
        
        #delta quest movement
        #delta_T = np.linalg.inv(self.last_pos) @ Tq
        #delta_right_movement=Tq[:3, 3] - self.last_pos[:3, 3]
        delta_T =  Tq[:3, 3] - self.last_pos[:3, 3]
        #delta_forward=np.dot(delta_quest, self.R_head_flat[:, 1])
        #delta_right=np.dot(delta_quest, self.R_head_flat[:, 2])
        #delta_vertical=delta_quest[1]


        dr=self.R_head_flat @ delta_T
        dr=R_rq @ dr 

        #Slow down movement
        #delta_T[:3, 3] *= 0.8 
        Tee= self.quest_ref[:3, 3]+dr#self.quest_ref @ delta_T
        self.get_logger().info("Delta movements: "+str(Tee))
        #self.get_logger().info("Original Pos: "+str(self.quest_ref[:3, :3]))
        #get position and set velocity to zero
        #R = frame.right_pose.matrix[:3, :3]
        #t = frame.right_pose.matrix[:3, 3]
        
        ee_des_pos = pin.SE3(self.quest_ref[:3, :3], Tee)#pin.SE3(R, t), prevhad: Tee[:3,3]
        #self.get_logger().info("Desired pose: "+str(pin.SE3ToXYZQUAT(ee_des_pos)))
        #self.last_pos=copy.deepcopy(frame.right_pose.matrix)
            
        #get IK and torques
        #q, dq = self.inverse_kinematics(ee_des_pos, ee_des_vel)
        #ddq = np.zeros(self.pin_model.nv)
        #u = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq)

        
        traj_point = TrajectoryPoint(
            id=0,
            time_ns=self.get_clock().now().nanoseconds, #+ int(0.2e9),
            robot_configuration=self.current_q,
            robot_velocity=np.zeros_like(self.current_q),
            robot_acceleration=np.zeros_like(self.current_q),
            robot_effort=np.zeros_like(self.current_q),
            end_effector_poses={self.ee_frame_name: pin.SE3ToXYZQUAT(ee_des_pos)},
        )

        traj_weights = TrajectoryPointWeights(
            w_robot_configuration=np.full_like(self.current_q, 1.0), #dtype=float), #self.w_q,
            w_robot_velocity=np.full_like(self.current_q, 0.1), #self.w_qdot,
            w_robot_acceleration= np.full_like(self.current_q, 0.000001), #self.w_qddot,
            w_robot_effort=np.full_like(self.current_q, 0.0008), #self.w_robot_effort,
            w_end_effector_poses={self.ee_frame_name: [30.0, 30.0, 30.0, 30.0, 30.0, 30.0] },#self.w_pose
        )
        weighted_traj_point=WeightedTrajectoryPoint(point=traj_point, weights=traj_weights)

        mpc_msg = weighted_traj_point_to_mpc_msg(weighted_traj_point)
        self.publisher_.publish(mpc_msg)
        self._marker_base.pose=mpc_msg.ee_inputs[0].pose
        self.get_logger().info("Marker base pos: "+str(self._marker_base.pose))
        self._target_pose_marker_pub.publish(self._marker_base)


def main(args=None):
    rclpy.init(args=args)
    node = QuestTrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__== '__main__':
    main()
    