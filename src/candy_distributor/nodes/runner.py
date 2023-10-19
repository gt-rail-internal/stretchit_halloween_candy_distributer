#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from xbox_ros.msg import joystick

def callback(data):
    print(data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('candy_distributor', anonymous=True)

    rospy.Subscriber("joystick_state", joystick, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
