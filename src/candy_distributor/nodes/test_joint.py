#!/usr/bin/env python3

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm

class TestJointsCommand(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def create_trajectory_point(self, positions):
        point = JointTrajectoryPoint()
        point.positions = positions
        return point

    def issue_single_joint_command(self, joint_name, position):
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = [joint_name]

        trajectory_point = self.create_trajectory_point([position])
        trajectory_goal.trajectory.points.append(trajectory_point)

        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo(f'Sent goal for joint = {joint_name}')
        self.trajectory_client.wait_for_result()

    def main(self):
        """
        Function to test each joint one by one.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'test_joints_command', 'test_joints_command', wait_for_first_pointcloud=False)

        joint_names = [
            'joint_gripper_finger_left',
            'joint_right_wheel',
            'joint_left_wheel',
            'joint_lift',
            'joint_arm_l3',
            'joint_arm_l2',
            'joint_arm_l1',
            'joint_arm_l0',
            'joint_wrist_yaw',
            'joint_head_pan',
            'joint_head_tilt'
        ]
        
        # The neutral position for each joint. Adjust as necessary.
        default_positions = [0.0] * len(joint_names)

        for joint_name, position in zip(joint_names, default_positions):
            input(f"Press Enter to test joint: {joint_name}")
            self.issue_single_joint_command(joint_name, position)

if __name__ == '__main__':
    try:
        node = TestJointsCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
