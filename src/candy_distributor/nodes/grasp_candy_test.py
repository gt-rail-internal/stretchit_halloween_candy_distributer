#!/usr/bin/env python3

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm

class MultiPointCommand(hm.HelloNode):
  def __init__(self):
    hm.HelloNode.__init__(self)

  def create_trajectory_point(self, positions):
    point = JointTrajectoryPoint()
    point.positions = positions
    return point

  def issue_multipoint_command(self):

    # Declare joint names of all Stretch Joints
    joint_names = [
        'joint_gripper_finger_left',  # Gripper only has 1DOF therefore we only choose one of the following options: 'joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture'
        'joint_lift',
        'wrist_extension',
        'joint_wrist_yaw',
        'joint_head_pan',
        'joint_head_tilt'
    ]

    # Initialize the positions we want to reach to grasp candy
    #home_position = [0.00043277803103309795, 0.9285284092581446, 0.0, 3.399684921151552, -1.8338199879850292, 0.0]
    reach_pre_grasp = [0.20261224819531565, 0.9285284092581446, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0]
    lower_arm_pre_grasp = [0.20261224819531565, 0.7513784372841598, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0]
    grasp_candy = [-0.12637118506165643, 0.751385067121114, 0.3, 0.16682041068256348, -1.8338199879850292, 0.0]
    lift_arm_grasp = [-0.12637118506165643, 0.9158684077447159, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0]
    reach_delivery = [-0.12637118506165643, 0.9158650563985633, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0]
    release_grasp = [0.20268437786715443,  0.9158650563985633, 0.4, 0.1674595693441825, -1.8338199879850292, 0.0]

    # Creation of the trajectory point data structure
    trajectory_points = [
        self.create_trajectory_point(reach_pre_grasp),
        self.create_trajectory_point(lower_arm_pre_grasp),
        self.create_trajectory_point(grasp_candy),
        self.create_trajectory_point(lift_arm_grasp),
        self.create_trajectory_point(reach_delivery),
        self.create_trajectory_point(release_grasp),
    ]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = joint_names

    for point in trajectory_points:
        trajectory_goal.trajectory.points.append(point)

    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def main(self):
    """
    Function that initiates the multipoint_command function.
    :param self: The self reference.
    """
    hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
    rospy.loginfo('issuing multipoint command...')
    self.issue_multipoint_command()
    time.sleep(2)


if __name__ == '__main__':
  try:
    node = MultiPointCommand()
    node.main()
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')