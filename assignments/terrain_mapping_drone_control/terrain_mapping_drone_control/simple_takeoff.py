#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')

        # Drone state
        self.state = State()
        self.connected = False

        # Desired position
        self.setpoint = PoseStamped()
        self.setpoint.pose.position.x = 0.0
        self.setpoint.pose.position.y = 0.0
        self.setpoint.pose.position.z = 2.5  # Target altitude

        # Subscribers and publishers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Internal state
        self.setpoint_counter = 0
        self.mode_set = False
        self.armed = False

        self.timer = self.create_timer(0.1, self.timer_cb)  # 10 Hz

    def state_cb(self, msg):
        self.state = msg
        self.connected = msg.connected

    def timer_cb(self):
        if not self.connected:
            self.get_logger().info("Waiting for FCU connection...")
            return

        # Publish setpoints continuously
        self.sp_pub.publish(self.setpoint)

        # Wait until 100 setpoints are published before switching mode
        if self.setpoint_counter < 100:
            self.setpoint_counter += 1
            return

        # Set OFFBOARD mode
        if not self.mode_set and self.state.mode != 'OFFBOARD':
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.mode_callback)
            self.mode_set = True

        # Arm the drone
        if not self.armed and self.state.mode == 'OFFBOARD' and not self.state.armed:
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_callback)
            self.armed = True

    def mode_callback(self, future):
        result = future.result()
        if result.mode_sent:
            self.get_logger().info("OFFBOARD mode set successfully.")
        else:
            self.get_logger().warn("Failed to set OFFBOARD mode.")

    def arm_callback(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info("Drone armed successfully.")
        else:
            self.get_logger().warn("Failed to arm drone.")

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

