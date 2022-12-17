#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

# Arduino
import serial

# Documentation: https://rethinkrobotics.github.io/intera_sdk_docs/5.0.4/intera_interface/html/intera_interface.gripper.Gripper-class.html#set_position
from intera_interface import gripper as robot_gripper

# EECS C106A - Final Project
# Team 17
# Authors: Grace Jung, Donald Le


def main():
    # ['close' position, 'open' position]
    # gripper_position_bounds = [0.0, 0.041667]

    rospy.init_node('gripper_control')

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    # Set maximum and minimum travel distance.
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # Close the right gripper
    print('Closing...')
    right_gripper.close()
    rospy.sleep(1.0)

    # Start reading in values from esp32 microcontroller and flex sensor.
    arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)
    def sensor_read():
        data = arduino.readline()
        return data

    def is_significant_change(position):
        tolerance = 0.001
        curr_pos = right_gripper.get_position()

        # Outside of tolerance bounds.
        return abs(curr_pos - position) > tolerance

    def convert_user_input(flex_value):
        empirical_vals = [3000, 4100]
        flex_value = round(flex_value, -1)
        position = (flex_value - empirical_vals[0]) / (empirical_vals[1] - empirical_vals[0])
        position = position * (right_gripper.MAX_POSITION - right_gripper.MIN_POSITION)
        position = position + right_gripper.MIN_POSITION
        
        # Update position so that high values of the flex_sensor correspond
        # to closing the gripper, i.e., lower values for the position.
        position = right_gripper.MAX_POSITION - position
        position = max(position, right_gripper.MIN_POSITION)
        return position


    while not rospy.is_shutdown():
        flex_value = sensor_read()

        if (flex_value):
            flex_value = float(flex_value)
            # print('Flex Sensor Reading:', flex_value)
            new_position = convert_user_input(flex_value)
            # print('new_position:', new_position)

            if is_significant_change(new_position):
                print('Setting position to:', new_position)
                # Setting the right gripper to a specific position
                right_gripper.set_position(new_position)
                print('-----DONE-----')
            
            flex_value = None
            new_position = None
    

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
