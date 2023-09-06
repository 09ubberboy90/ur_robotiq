import math
import time
import socket

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
        self.HOST="163.220.51.116" #my ur ip 
        self.port=63352 #PORT used by robotiq gripper

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
        self.opened_value = 0.0
        self.closed_value = 255.0
        self.last_trigger = False

    def listener_callback(self, msg:JointState):
        for name, pose in zip(msg.name, msg.position):
            self.joint_states[name] = pose
        
        out_msg = Float64MultiArray()
         
        for el in self.joint_names:
            out_msg.data.append(self.joint_states[el])
        

        if math.isclose(abs(self.joint_states["finger_joint"]), 0.7, abs_tol=0.1):
            trigger = True
        else:
            trigger = False
        
        if trigger and not self.last_trigger:
            self.send_gripper_command(self.closed_value)
        elif not trigger and self.last_trigger:
            self.send_gripper_command(self.opened_value)

        self.last_trigger = trigger

        self.publisher.publish(out_msg)

    def send_gripper_command(self, value):
        print(f"Sending gripper command: {value}")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.HOST, self.port))
            s.sendall(b"SET GTO 0\n")
            s.sendall(f"SET POS {int(value)}\n".encode())
            s.sendall(b"SET GTO 1\n")
            

def main(args=None):
    rclpy.init(args=args)

    node = JointController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
