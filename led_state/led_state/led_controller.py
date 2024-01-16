#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msg.msg import StateAndBattery 


class LedBat_Publisher(Node):

    def __init__(self):
        super().__init__('ledbat_publisher')
        self.publisher_ = self.create_publisher(StateAndBattery, 'LedBat', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ledbat_data = {"state": 0, "battery": 0}
        
        print(">> Type 's <uint32>' to change the state. Range [0, 15]\n>> Type 'b <uint32>' to change battery percentage. Range [0, 100]\n>> Type 'q' to quit")    

    def timer_callback(self):
        input_data = input(">> ").split(" ")
        if input_data[0] == 'b':
            self.set_battery(int(input_data[1]))
        elif input_data[0] == 's':
            self.set_state(int(input_data[1]))
        msg = StateAndBattery()
        msg.state = self.ledbat_data["state"]
        msg.battery = self.ledbat_data["battery"]
        self.publisher_.publish(msg)
        self.get_logger().info('State: "%d"' % msg.state)

    def set_state(self, value: int):
        self.ledbat_data["state"] = value

    def set_battery(self, value: int):
        self.ledbat_data["battery"] = value


def main(args=None):
    rclpy.init(args=args)

    LB = LedBat_Publisher()

    input_data = ['x', 'x']
    rclpy.spin(LB)

    print(">> Type 's <uint32>' to change the state. Range [0, 15]\n>> Type 'b <uint32>' to change battery percentage. Range [0, 100]\n>> Type 'q' to quit")
    while(input_data[0] != "q"):
        input_data = input(">> ").split(" ")
        if input_data[0] == 'b':
            LB.set_battery(int(input_data[1]))
        elif input_data[0] == 's':
            LB.set_state(int(input_data[1]))
            
    LB.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
