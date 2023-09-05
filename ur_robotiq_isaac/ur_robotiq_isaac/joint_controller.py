import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Int32
import numpy as np

class JointController(Node):

    def __init__(self):
        super().__init__('ur_joint_controller')
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        # self.motor_pub = self.create_publisher(JointState, 't42_motor_control', 10)

        self.subscription = self.create_subscription(
            JointState,
            'joint_states_sim',
            self.listener_callback,
            10)
        
        # self.right_gripper = self.create_subscription(Int32, 'right_offset', self.right_gripper_callback, 10)
        # self.left_gripper = self.create_subscription(Int32, 'left_offset', self.left_gripper_callback, 10)

        self.joint_states = {}
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        # self.gripper_names = {
        #     "left": "swivel_1_to_finger_1_1",
        #     "right": "swivel_2_to_finger_2_1",
        # }
        # self.closed_value = 90 
        # ## Run adjust offset node
        # ## Its in degrees
        # self.gripper_offset = {
        #     "left": 20.0,
        #     "right": 10.0,
        # }

    def right_gripper_callback(self, msg):
        self.gripper_offset["right"] = msg.data
    
    def left_gripper_callback(self, msg):
        self.gripper_offset["left"] = msg.data


    def listener_callback(self, msg:JointState):
        for name, pose in zip(msg.name, msg.position):
            self.joint_states[name] = pose
        
        out_msg = Float64MultiArray()
         
        for el in self.joint_names:
            out_msg.data.append(self.joint_states[el])
        
        self.publisher.publish(out_msg)
        # outgoing = JointState()
        # for key, el in self.gripper_names.items():
        #     outgoing.name.append(key)
        #     requested_degree = self.joint_states[el]*180/np.pi
        #     if requested_degree > 90:
        #         requested_degree = 120
        #     outgoing.position.append(requested_degree + self.gripper_offset[key])
        # self.motor_pub.publish(outgoing)


def main(args=None):
    rclpy.init(args=args)

    node = JointController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
