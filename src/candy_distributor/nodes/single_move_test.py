#!/usr/bin/env python3

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm

class SequentialPointCommand(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def create_trajectory_point(self, positions):
        point = JointTrajectoryPoint()
        point.positions = positions
        return point

    def issue_singlepoint_command(self, positions):
        # Declare joint names of all Stretch Joints
        joint_names = [
            'joint_gripper_finger_left',
            'joint_lift',
            'wrist_extension',
            'joint_wrist_yaw',
            'joint_head_pan',
            'joint_head_tilt'
        ]

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = joint_names

        trajectory_point = self.create_trajectory_point(positions)
        trajectory_goal.trajectory.points.append(trajectory_point)

        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent goal = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()

    def main(self):
        hm.HelloNode.main(self, 'sequentialpoint_command', 'sequentialpoint_command', wait_for_first_pointcloud=False)

        poses = {
            "home": [0.00043277803103309795, 0.9285284092581446, 0.0, 3.399684921151552, -1.8338199879850292, 0.0],
            "pre_grasp_1": [0.20261224819531565, 0.9285284092581446, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0],
            "pre_grasp_2": [0.20261224819531565, 0.7513784372841598, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0],
            'grasp_1' : [-0.12637118506165643, 0.751385067121114, 0.3, 0.16682041068256348, -1.8338199879850292, 0.0],
            'grasp_2' : [-0.12637118506165643, 0.9158684077447159, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0],
            'pre_delivery' : [-0.12637118506165643, 0.9158650563985633, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0],
            'delivery' : [0.20268437786715443,  0.9158650563985633, 0.4, 0.1674595693441825, -1.8338199879850292, 0.0],
            # ... add all other poses similarly
        }

        for name, position in poses.items():
            rospy.loginfo(f"Moving to pose: {name}")
            self.issue_singlepoint_command(position)
            time.sleep(2)

        rospy.loginfo("Finished all poses.")

if __name__ == '__main__':
    try:
        node = SequentialPointCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
