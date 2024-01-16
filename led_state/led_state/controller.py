#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msg.msg import StateAndBattery 

class StateListener(Node):
    
    def __init__(self):
        super().__init__('state_listener')
        self.subscription = self.create_subscription(
            StateAndBattery,
            'LedBat',
            self.listener_callback,
            10)
        self._buffer = {"state": 0, "battery": 0}
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.set_buffer(msg.state, msg.battery)
        if self._buffer["battery"] == 15:
            self.get_logger().info(f'Executing Charging state, battery percentage: {self._buffer["battery"]}')
        else:
            self.get_logger().info(f'Executing new state: {self._buffer["state"]}')
        with open("/home/refro/ros2_ws/src/led_state/led_msg.txt", 'w') as f:
            f.write(f'state:{self._buffer["state"]}\nbattery:{self._buffer["battery"]}')

    def set_buffer(self, s, b):
        self._buffer["state"] = s
        self._buffer["battery"] = b

def main(args=None):
    rclpy.init(args=args)

    LB = StateListener()
    rclpy.spin(LB)
            
    LB.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()