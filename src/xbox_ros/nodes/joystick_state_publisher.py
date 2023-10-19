#!/usr/bin/python3
from __future__ import print_function
import stretch_body.xbox_controller as xc
import stretch_body.robot as rb
from stretch_body.hello_utils import *
import os
import time
import argparse
import random
import math
from xbox_ros.msg import joystick
import rospy

print_stretch_re_use()

parser = argparse.ArgumentParser(description=
                                 'Jog the robot from an XBox Controller  \n' +
                                 '-------------------------------------\n' +
                                 'Left Stick X:\t Rotate base \n' +
                                 'Left Stick Y:\t Translate base \n' +
                                 'Right Trigger:\t Fast base motion \n' +
                                 'Right Stick X:\t Translate arm \n' +
                                 'Right Stick Y:\t Translate lift \n' +
                                 'Left Button:\t Rotate wrist CCW \n' +
                                 'Right Button:\t Rotate wrist CW \n' +
                                 'A/B Buttons:\t Close/Open gripper \n' +
                                 'Left/Right Pad:\t Head Pan \n' +
                                 'Top/Bottom Pad:\t Head tilt \n' +
                                 'Y Button :\t Go to stow position \n ' +
                                 'Start Button:\t Home robot \n ' +
                                 'Back Button (2 sec):\t Shutdown computer \n ' +
                                 '-------------------------------------\n',
                                 formatter_class=argparse.RawTextHelpFormatter)

args = parser.parse_args()

def convert_to_joystick_ros_msg(controller_state): # Credits to: ChatGPT
    # Create a new instance of the ControllerState ROS message
    controller_state_msg = joystick()

    # Assign values from the input dictionary to the message fields
    controller_state_msg.middle_led_ring_button_pressed = controller_state['middle_led_ring_button_pressed']
    controller_state_msg.left_stick_x = controller_state['left_stick_x']
    controller_state_msg.left_stick_y = controller_state['left_stick_y']
    controller_state_msg.right_stick_x = controller_state['right_stick_x']
    controller_state_msg.right_stick_y = controller_state['right_stick_y']
    controller_state_msg.left_stick_button_pressed = controller_state['left_stick_button_pressed']
    controller_state_msg.right_stick_button_pressed = controller_state['right_stick_button_pressed']
    controller_state_msg.bottom_button_pressed = controller_state['bottom_button_pressed']
    controller_state_msg.top_button_pressed = controller_state['top_button_pressed']
    controller_state_msg.left_button_pressed = controller_state['left_button_pressed']
    controller_state_msg.right_button_pressed = controller_state['right_button_pressed']
    controller_state_msg.left_shoulder_button_pressed = controller_state['left_shoulder_button_pressed']
    controller_state_msg.right_shoulder_button_pressed = controller_state['right_shoulder_button_pressed']
    controller_state_msg.select_button_pressed = controller_state['select_button_pressed']
    controller_state_msg.start_button_pressed = controller_state['start_button_pressed']
    controller_state_msg.left_trigger_pulled = controller_state['left_trigger_pulled']
    controller_state_msg.right_trigger_pulled = controller_state['right_trigger_pulled']
    controller_state_msg.bottom_pad_pressed = controller_state['bottom_pad_pressed']
    controller_state_msg.top_pad_pressed = controller_state['top_pad_pressed']
    controller_state_msg.left_pad_pressed = controller_state['left_pad_pressed']
    controller_state_msg.right_pad_pressed = controller_state['right_pad_pressed']

    return controller_state_msg

def check_usb_devices(wait_timeout=5):
    hello_devices = ['hello-wacc',
                     'hello-motor-left-wheel',
                     'hello-pimu',
                     'hello-dynamixel-head',
                     'hello-dynamixel-wrist',
                     'hello-motor-arm',
                     'hello-motor-right-wheel',
                     'hello-motor-lift']

    print('Waiting for all the hello* devices ...')
    all_found = True
    for dev in hello_devices:
        if not wait_till_usb(dev, wait_timeout):
            all_found = False
    if all_found:
        print('Found all hello* devices.')
    return all_found


def wait_till_usb(usb, wait_timeout):
    s_ts = time.time()
    while time.time() - s_ts <= wait_timeout:
        devices = os.listdir('/dev')
        hello_devs = [dev for dev in devices if 'hello' in dev]
        if usb in hello_devs:
            return True
    print('{} device not found.'.format(usb))
    return False


def do_double_beep(robot):
    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)
    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)

# ######################### MAIN ########################################
use_head_mapping = True
use_dex_wrist_mapping = False
use_stretch_gripper_mapping = True


def main():
    global use_head_mapping, use_dex_wrist_mapping, use_stretch_gripper_mapping
    xbox_controller = xc.XboxController()
    xbox_controller.start()
    check_usb_devices(wait_timeout=5)
    rospy.init_node('joystick_input', anonymous=True)
    joystick_pub = rospy.Publisher('joystick_state', joystick, queue_size=10)
    # robot = rb.Robot()
    rate = rospy.Rate(60)
    try:
        while not rospy.is_shutdown():
            controller_state = xbox_controller.get_state()
            # print(controller_state)
            joystick_state_msg = convert_to_joystick_ros_msg(controller_state)
            joystick_pub.publish(joystick_state_msg)

            rate.sleep()
    except (ThreadServiceExit, KeyboardInterrupt, SystemExit):
        # robot.stop()
        xbox_controller.stop()


if __name__ == "__main__":
    main()
