#!/usr/bin/env python3

# Led display for direction
# from multiprocessing import shared_memory
from led_display.spi_module import SPItoWS

#ROS
# import rclpy
# from rclpy.node import Node

# from led_battery.msg import state_and_battery 

import time
import math
import sys

import rclpy
from rclpy.node import Node

from control_msg.msg import StateAndBattery 

# class StateListener(Node):
    
#     def __init__(self):
#         super().__init__('state_listener')
#         self.subscription = self.create_subscription(
#             StateAndBattery,
#             'LedBat',
#             self.listener_callback,
#             10)
#         self._buffer = {"state": 0, "battery": 0}
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.set_buffer(msg.state, msg.battery)
#         if self._buffer["battery"] == 15:
#             self.get_logger().info(f'Executing Charging state, battery percentage: {self._buffer["battery"]}')
#         else:
#             self.get_logger().info(f'Executing new state: {self._buffer["state"]}')
#         with open("/home/refro/ros2_ws/src/led_state/led_msg.txt", 'w') as f:
#             f.write(f'state:{self._buffer["state"]}\nbattery:{self._buffer["battery"]}')

#     def set_buffer(self, s, b):
#         self._buffer["state"] = s
#         self._buffer["battery"] = b

# def main(args=None):
#     rclpy.init(args=args)

#     LB = StateListener()
#     rclpy.spin(LB)
            
#     LB.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

class LedInfo():
    def __init__(self):

        self.testing = True

        self.MAX_BRIGHT = 255

        self.LED_STRIP = 18
        self.TOTAL_LEDS = 4*self.LED_STRIP

        self.FL = 0
        self.FR = self.LED_STRIP
        self.BR = 2 * self.LED_STRIP
        self.BL = 3 * self.LED_STRIP

        self.COLORS = {"white": [x*self.MAX_BRIGHT//255 for x in (255, 255, 255)],
                "blue": [x*self.MAX_BRIGHT//255 for x in (0, 0, 255)],
                "red": [x*self.MAX_BRIGHT//255 for x in (255, 0, 0)],
                "yellow": [x*self.MAX_BRIGHT//255 for x in (255, 255, 0)],
                "orange": [x*self.MAX_BRIGHT//255 for x in (255, 102, 0)],
                "green": [x*self.MAX_BRIGHT//255 for x in (0, 255, 0)]}

        self.spi = SPItoWS(self.TOTAL_LEDS)

        self.tic_blink = 0
        self.blink_status = False
        self.tic_stream = 0
        self.stream_led = 0

        self.tic_charge = 0
        self.charge_led = 0.0
        # self.buffer = [0, 0]

        self.ros_timer = 0.0

def clear_leds(info: LedInfo, initial_led: int, leds_amount: int = 18):
    for i in range(initial_led, initial_led + leds_amount):
        info.spi.RGBto3Bytes(i, 0, 0, 0)
    info.spi.LED_show()

def fill_leds(info: LedInfo, initial_led: int, leds_amount: int = 18, color: str = "white"):
    # TODO update so that you can change colour
    color_ = info.COLORS[color]
    for i in range(initial_led, leds_amount + initial_led):
        info.spi.RGBto3Bytes(i, *color_)
    info.spi.LED_show()

def blink_leds(info: LedInfo, initial_led, leds_amount: int, color: str = "white"):
    # TODO if the leds amount is not the same for every strip of leds, we need to make sure it becomes a list too
    if isinstance(initial_led, int):
        initial_led = [initial_led]
    if time.time() - info.tic_blink > 0.25: # TODO maybe change to absolute
        if info.blink_status is False:
            for item in initial_led:
                fill_leds(info, item, leds_amount, color)
            info.blink_status = True
        else:
            for item in initial_led:
                clear_leds(info, item, leds_amount)
            info.blink_status = False
        # print(f"blinked {info.blink_status}\n")
        info.tic_blink = time.time()

def stream(info: LedInfo, initial_led, leds_amount: int, direction = True, color: str = "white"):
    # TODO if the leds amount is not the same for every strip of leds, we need to make sure it becomes a list too
    if isinstance(initial_led, int):
        initial_led = [initial_led] # make sure code works with list of initial_leds and 
    if isinstance(direction, int):
        direction = [direction]
    if info.stream_led >= leds_amount:
        # print(info.tic_blink, info.tic_stream, time.time(), leds_amount)
        if time.time() - info.tic_stream > 0.18:
            # print('full stream\n')
            for item in initial_led:
                clear_leds(info, item, leds_amount)
            info.stream_led = 0
            info.tic_stream = time.time()
            if len(direction) == 2: # This only works because anytime there are two leds that stream, there is also a led that blinks
                info.tic_blink = info.tic_stream-1 # This makes the blink and stream synchronised, place stream BEFORE blink in funct
    else:
        if time.time() - info.tic_stream > 0.01389:
            for i in range(len(initial_led)):
                if direction[i] is True:
                    info.spi.RGBto3Bytes(initial_led[i]+info.stream_led, *info.COLORS[color])
                else:
                    info.spi.RGBto3Bytes(initial_led[i]+leds_amount-info.stream_led-1, *info.COLORS[color])
            info.stream_led+=1
            info.tic_stream = time.time()
            info.spi.LED_show()
    
def stream_cw(info: LedInfo, initial_led, leds_amount: int, color: str = "white"):
    # TODO if the leds amount is not the same for every strip of leds, we need to make sure it becomes a list too
    # Function is outdated because of stream() but it is slightly faster
    if isinstance(initial_led, int):
        initial_led = [initial_led] # make sure code works with list of initial_leds and 
    if info.stream_led >= leds_amount:
        if time.time() - info.tic_stream > 0.25:
            for item in initial_led:
                clear_leds(info, item, leds_amount)
            info.stream_led = 0
            info.tic_stream = time.time()
    else:
        if time.time() - info.tic_stream > 0.01389:
            for item in initial_led:
                info.spi.RGBto3Bytes(item+info.stream_led, *info.COLORS[color])
            info.stream_led+=1
            info.tic_stream = time.time()
            info.spi.LED_show()

def stream_ccw(info: LedInfo, initial_led, leds_amount: int, color: str = "white"):
    # TODO if the leds amount is not the same for every strip of leds, we need to make sure it becomes a list too
    if isinstance(initial_led, int):
        initial_led = [initial_led] # make sure code works with list of initial_leds and
    if info.stream_led > leds_amount:
        if time.time() - info.tic_stream > 0.25:
            for item in initial_led:
                clear_leds(info, item, leds_amount)
            info.stream_led = 0
            info.tic_stream = time.time()
            return
    else:
        if time.time() - info.tic_stream > 0.01389:
            for item in initial_led:
                info.spi.RGBto3Bytes(item+leds_amount-info.stream_led-1, *info.COLORS[color])
            info.stream_led+=1
            info.tic_stream = time.time()
            info.spi.LED_show()

def funct_standby(info: LedInfo): #0
    fill_leds(info, info.FL, info.TOTAL_LEDS, "white")

def funct_move_man(info: LedInfo): #1
    fill_leds(info, info.FL, info.TOTAL_LEDS, "blue")

def funct_move_auto(info: LedInfo): #2
    blink_leds(info, info.FL, info.TOTAL_LEDS, "blue")
    
def funct_auto_right(info: LedInfo): #3
    stream(info, [info.FR, info.BR], info.LED_STRIP, [True, False], "blue")
    blink_leds(info, [info.FL, info.BL], info.LED_STRIP, "blue")

def funct_auto_left(info: LedInfo): #4
    stream(info, [info.FL, info.BL], info.LED_STRIP, [False, True], "blue")
    blink_leds(info, [info.FR, info.BR], info.LED_STRIP, "blue")

def funct_obst_slow(info: LedInfo): #5
    blink_leds(info, info.FL, info.TOTAL_LEDS, "yellow")

def funct_obst_stop(info: LedInfo): #6
    fill_leds(info, info.FL, info.TOTAL_LEDS, "yellow")

def funct_obst_right(info: LedInfo): #7
    stream(info, [info.FR, info.BR], info.LED_STRIP, [True, False], "yellow")
    blink_leds(info, [info.FL, info.BL], info.LED_STRIP, "yellow")

def funct_obst_left(info: LedInfo): #8
    stream(info, [info.FL, info.BL], info.LED_STRIP, [False, True], "yellow")
    blink_leds(info, [info.FR, info.BR], info.LED_STRIP, "yellow")

def funct_auto_fail(info: LedInfo): #9
    blink_leds(info, info.FL, info.TOTAL_LEDS, "red")

def funct_alert_stop(info: LedInfo): #10
    fill_leds(info, info.FL, info.TOTAL_LEDS, "red")

def funct_alert_one(info: LedInfo): #11
    stream_ccw(info, info.FL, info.LED_STRIP, "red")
    fill_leds(info, info.FR, info.LED_STRIP, "red")
    fill_leds(info, info.BR, info.LED_STRIP, "red")
    fill_leds(info, info.BL, info.LED_STRIP, "red")

def funct_alert_two(info: LedInfo): #12
    fill_leds(info, info.FL, info.LED_STRIP, "red")
    stream_cw(info, info.FR, info.LED_STRIP, "red")
    fill_leds(info, info.BR, info.LED_STRIP, "red")
    fill_leds(info, info.BL, info.LED_STRIP, "red")

def funct_alert_three(info: LedInfo): #13
    fill_leds(info, info.FL, info.LED_STRIP, "red")
    fill_leds(info, info.FR, info.LED_STRIP, "red")
    stream_ccw(info, info.BR, info.LED_STRIP, "red")
    fill_leds(info, info.BL, info.LED_STRIP, "red")

def funct_alert_four(info: LedInfo): #14
    fill_leds(info, info.FL, info.LED_STRIP, "red")
    fill_leds(info, info.FR, info.LED_STRIP, "red")
    fill_leds(info, info.BR, info.LED_STRIP, "red")
    stream_cw(info, info.BL, info.LED_STRIP, "red")

def charge_amount(info, full_leds, color): # 15: Additional function for numerical gradation 
    if (full_leds == 0):
        for i in range(info.TOTAL_LEDS):
            _color = math.sin(2*math.pi/info.TOTAL_LEDS*6 * (i + info.charge_led))
            _color = int(_color*255/2 + 255/2)
            info.spi.RGBto3Bytes(i, 0, _color, 0)
    else:
        if full_leds == info.LED_STRIP:
            full_leds -= 1
        for i in range(info.LED_STRIP):
            if (i < full_leds):
                for strip in range(4):
                    info.spi.RGBto3Bytes(strip*info.LED_STRIP+i, 0, 0, 0)
            else:
                _color = math.sin(2*math.pi/info.LED_STRIP*1.5 * (i + info.charge_led))
                _color = [int(_color*x/2 + x/2) for x in color]
                for strip in range(4):
                    info.spi.RGBto3Bytes(strip*info.LED_STRIP+i, *_color)
    info.spi.LED_show()
    info.charge_led += 0.5
    info.tic_charge = time.time()

def funct_charge(info: LedInfo, buffer): # 15
    if time.time() - info.tic_charge > 0.05:
        if buffer > 95: #Full
            charge_amount(info, 0, 'green')
        else:
            leds_off = min(info.LED_STRIP-1, info.LED_STRIP-math.ceil(buffer*info.LED_STRIP/100)+1)
            if buffer < 50:
                _color = (255, 255*buffer//50, 0)
            else:
                _color = (255-255*(buffer-50)//50, 255, 0)
            charge_amount(info, leds_off, _color)

def main(args=None):
    try:        
        LD = LedInfo()
        # spi = SPItoWS(LD.TOTAL_LEDS)

        ros_timer = time.time()
        clear_leds(LD, 0, LD.TOTAL_LEDS)

        temp_char = 0 # for now it is int
        function_cache = funct_standby

        MODES = {"STANDBY": 0,
            "MOVE_MANUALLY": 1,
            "AUTONOMOUS": 2,
            "AUTONOMOUS_RIGHT": 3,
            "AUTONOMOUS_LEFT": 4,
            "OBSTACLE_SLOW": 5,
            "OBSTACLE_STOP": 6,
            "OBSTACLE_RIGHT": 7,
            "OBSTACLE_LEFT": 8,
            "AUTONOMOUS_FAIL": 9,
            "ALERT_STOP": 10,
            "ALERT_ONE": 11,
            "ALERT_TWO": 12,
            "ALERT_THREE": 13,
            "ALERT_FOUR": 14,
            "CHARGE": 15}

        _buffer = {'state': 0, 'battery': 0}
        temp_char = 0

        def callback(msg):
            _buffer['state'] = msg.state
            _buffer['battery'] = msg.battery

        rclpy.init(args=args)
        ros_node = rclpy.create_node('led_control_subs')
        ros_subscriber = ros_node.create_subscription(StateAndBattery, 'led_control', callback, 10) 
    
        if not rclpy.ok():
            print('problem with rclpy occured')
        else:
            print('Node created!')

        while(1):
            try:
                temp_time = time.time()
                if temp_time - 0.5 > ros_timer:
                    rclpy.spin_once(ros_node)
                    # print(_buffer)
                    ros_timer = temp_time
            except IndexError:
                print('something is wrong with msg.txt file')
            if temp_char != _buffer["state"] and _buffer['state'] in MODES.values():
                clear_leds(LD, LD.FL, LD.TOTAL_LEDS) # clear all the leds
                temp_char = _buffer["state"]

                LD.tic_blink = time.time()
                LD.tic_stream = LD.tic_blink
                LD.blink_status = True

                # if LD.testing:
                #     print(f'New state: {temp_char}\n')
                if temp_char == MODES["STANDBY"]:
                    funct_standby(LD)
                    function_cache = funct_standby
                elif temp_char == MODES["MOVE_MANUALLY"]:
                    funct_move_man(LD)
                    function_cache = funct_move_man
                elif temp_char == MODES["AUTONOMOUS"]:
                    funct_move_auto(LD)
                    function_cache = funct_move_auto
                elif temp_char == MODES["AUTONOMOUS_RIGHT"]:   
                    funct_auto_right(LD)
                    function_cache = funct_auto_right
                elif temp_char == MODES["AUTONOMOUS_LEFT"]:
                    funct_auto_left(LD)
                    function_cache = funct_auto_left
                elif temp_char == MODES["OBSTACLE_SLOW"]:
                    funct_obst_slow(LD)
                    function_cache = funct_obst_slow
                elif temp_char == MODES["OBSTACLE_STOP"]:
                    funct_obst_stop(LD)
                    function_cache = funct_obst_stop
                elif temp_char == MODES["OBSTACLE_RIGHT"]: 
                    funct_obst_right(LD)
                    function_cache = funct_obst_right
                elif temp_char == MODES["OBSTACLE_LEFT"]:
                    funct_obst_left(LD)
                    function_cache = funct_obst_left
                elif temp_char == MODES["AUTONOMOUS_FAIL"]:
                    funct_auto_fail(LD)
                    function_cache = funct_auto_fail
                elif temp_char == MODES["ALERT_STOP"]:
                    funct_alert_stop(LD)
                    function_cache = funct_alert_stop
                elif temp_char == MODES["ALERT_ONE"]: 
                    funct_alert_one(LD)
                    function_cache = funct_alert_one
                elif temp_char == MODES["ALERT_TWO"]:
                    funct_alert_two(LD)
                    function_cache = funct_alert_two
                elif temp_char == MODES["ALERT_THREE"]:
                    funct_alert_three(LD)
                    function_cache = funct_alert_three
                elif temp_char == MODES["ALERT_FOUR"]: 
                    funct_alert_four(LD)
                    function_cache = funct_alert_four
                elif temp_char == MODES["CHARGE"]:
                    LD.tic_charge = time.time()
                    funct_charge(LD, _buffer['battery'])
                    function_cache = funct_charge
                else:
                    print('something sus is going on')
                    function_cache(LD)

            if temp_char == 15:
                function_cache(LD, _buffer['battery'])
            else:
                function_cache(LD)

            time.sleep(0.001)

    except KeyboardInterrupt:
        # print(E)
        print("exit gracefully")
        clear_leds(LD, 0, LD.TOTAL_LEDS)
        del LD
        rclpy.shutdown()

if __name__ == '__main__':
    main()