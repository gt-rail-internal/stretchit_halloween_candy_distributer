#!/usr/bin/env python3
# Base code taken from grasp_candy.py 

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
# import tf
import tf2_ros
import random

from candy_distributor.srv import candy_pick_srv, candy_pick_srvResponse

class ArucoGraspCandy(hm.HelloNode):
  def __init__(self):
    hm.HelloNode.__init__(self)
    # self.tf_listener = tf.TransformListener()
    # self.tf_buffer__ = tf2_ros.Buffer()
    # self.listener__ = tf2_ros.TransformListener(self.tf_buffer__)
    
    # Declare joint names of all Stretch Joints
    self.joint_names = [
        'joint_gripper_finger_left',  # Gripper only has 1DOF therefore we only choose one of the following options: 'joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture'
        'joint_lift',
        'wrist_extension',
        'joint_wrist_yaw',
        'joint_head_pan',
        'joint_head_tilt',
    ]
    self.camera_joint_names = ['joint_head_pan','joint_head_tilt']
    self.height_offset = 0.02 #+0.3
    self.wrist_extension_offset = -0.2
    self.head_pan = -1.5
    self.head_tilt = -0.5    
    self.wrist_yaw_stow = 2.5
    self.wrist_straight = 0.1674595693441825
    self.candy_pick_service = rospy.Service('candy_pick_service', candy_pick_srv, self.candy_pick_service_callback)

  def candy_pick_service_callback(self,req):
    rospy.loginfo("received request")
    if(req.candy_no in [1,2,3]):
      # self.aruco_grasp_candy_command(req.candy_no)
      self.camera_scan_non_blocking()
      return candy_pick_srvResponse(True)
    else:
      return candy_pick_srvResponse(False)
  def create_trajectory_point(self, positions):
    point = JointTrajectoryPoint()
    point.positions = positions
    return point
  def camera_point_for_candy(self):
    camera_pos = [self.head_pan, self.head_tilt]
    
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = self.camera_joint_names
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'
    trajectory_goal.trajectory.points.append(self.create_trajectory_point(camera_pos))
    self.trajectory_client.send_goal(trajectory_goal)
    self.trajectory_client.wait_for_result()

  def reach_candy_delivery_pose(self):
    # Initialize the positions we want to reach to grasp candy
    lift_arm_grasp = [-0.12637118506165643, 0.9158684077447159, 0.3, 0.1674595693441825, self.head_pan, self.head_tilt]
    reach_delivery = [-0.12637118506165643, 0.9158650563985633, 0.4, 0.1674595693441825, self.head_pan, self.head_tilt]
    release_grasp = [0.20268437786715443,  0.9158650563985633, 0.4, 0.1674595693441825, self.head_pan, self.head_tilt]

    # Creation of the trajectory point data structure
    trajectory_points = [
        self.create_trajectory_point(lift_arm_grasp),
        self.create_trajectory_point(reach_delivery),
        self.create_trajectory_point(release_grasp)
    ]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = self.joint_names

    for point in trajectory_points:
        trajectory_goal.trajectory.points.append(point)

    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent trajectory')
    self.trajectory_client.wait_for_result()
  

  def stow_for_visibility(self):
    # Function to move the arm to avoid blocking the aruco detection
    # Initialize the positions we want to reach to grasp candy
    print("stowing")
    stow_joints = ['joint_lift','wrist_extension','joint_wrist_yaw']
    move_up_pt = [0.9, 0.4, 0.1674595693441825]
    tuck_arm = [0.9, 0.1, 2.5]
    move_low = [0.7, 0.1 ,  2.5]

    # Creation of the trajectory point data structure
    trajectory_points = [
        self.create_trajectory_point(move_up_pt),
        self.create_trajectory_point(tuck_arm),
        self.create_trajectory_point(move_low)
    ]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = stow_joints

    for point in trajectory_points:
        trajectory_goal.trajectory.points.append(point)

    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    rospy.loginfo('Sent trajectory')
    self.trajectory_client.wait_for_result()
  
  def wrap_height(self,height):
    return max(min(height,1.0),0.3)
  def wrap_extension(self,extension):
    return max(min(extension,0.5),0.0)
  def wrap_pan(self,pan):
    return max(min(pan,1.5),-4.0)
  def wrap_tilt(self,tilt):
    return max(min(tilt,0.4),-1.0)
  def camera_scan_non_blocking(self):
    # creating an array of pan values from -1.5 to 1.5 with random increments in between 0.1 to 1.5
    scan_min = -1.0
    scan_max = 1
    head_tilt = -0.2
    head_pan = -1.5
    pan_rand_weight = 0.2
    pan_rand_const = 0.1
    tilt_rand_weight = 2.0
    pan_rand_buffer_size = 10
    tilt_rand_buffer_size = 30
    head_pan_array = [head_pan+scan_min]
    random_queue = []
    for i in range(pan_rand_buffer_size):
      random_queue.append(random.random())
    for i in range(100):
      rand_no = random.random()
      random_queue.pop()
      random_queue.append(rand_no)
      rand = sum(random_queue)/len(random_queue)
      rand=rand*pan_rand_weight+pan_rand_const
      head_pan_array.append(head_pan_array[-1]+rand)
      if(head_pan_array[-1]>head_pan+scan_max):
        break
    for i in range(100):
      rand_no = random.random()
      random_queue.pop()
      random_queue.append(rand_no)
      rand = sum(random_queue)/len(random_queue)
      rand=rand*pan_rand_weight+pan_rand_const
      head_pan_array.append(head_pan_array[-1]-rand)
      if(head_pan_array[-1]<head_pan+scan_min):
        break
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = ['joint_head_pan','joint_head_tilt']
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'
    random_queue = []
    for i in range(tilt_rand_buffer_size):
      random_queue.append(random.random()-0.5)
    print('head_array_length:',len(head_pan_array))
    for pan in head_pan_array:
      random_no = random.random()-0.5
      random_queue.pop()
      random_queue.append(random_no)
      random_tilt = sum(random_queue)/len(random_queue)
      random_tilt = random_tilt*tilt_rand_weight
      trajectory_goal.trajectory.points.append(self.create_trajectory_point([self.wrap_pan(pan),self.wrap_tilt(head_tilt-random_tilt)]))
    trajectory_goal.trajectory.points.append(self.create_trajectory_point([self.wrap_pan(head_pan),self.wrap_tilt(head_tilt)]))
    
    # trajectory_goal.trajectory.points.append(self.create_trajectory_point(camera_left_pose))
    # trajectory_goal.trajectory.points.append(self.create_trajectory_point(camera_right_pose))
    self.trajectory_client.send_goal(trajectory_goal)
    return trajectory_goal
  def scan_and_get_aruco(self,candy_frame_id):
    self.camera_point_for_candy()
    traj_goal = self.camera_scan_non_blocking()
    # get aruco
    rospy.logwarn("waiting for transform. Blocking till aruco found")
    try:
        transform = self.get_tf('base_link',candy_frame_id)
        wrist_extension = -1*transform.transform.translation.y + self.wrist_extension_offset
        height = transform.transform.translation.z
        move_base_distance = transform.transform.translation.x
        # height = 0.8
        height+=self.height_offset
        height = max(min(height,1.0),0.3)
        print("height",height)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Could not get transform from 'base_link' to "+str(candy_frame_id))
        return
    
    self.trajectory_client.wait_for_result()
    return [move_base_distance,height,wrist_extension]
  
  def aruco_grasp_candy_command(self,candy_no):
    candy_frame_id = 'candy'+str(candy_no)
    rospy.loginfo("candy frame id: "+str(candy_frame_id))
    self.camera_point_for_candy()
    # rospy.logwarn("waiting for transform. Blocking till aruco found")
    # try:
    #     transform = self.get_tf('base_link',candy_frame_id)
    #     wrist_extension = -1*transform.transform.translation.y + self.wrist_extension_offset
    #     height = transform.transform.translation.z
    #     move_base_distance = transform.transform.translation.x
    #     # height = 0.8
    #     height+=self.height_offset
    #     height = max(min(height,1.0),0.3)
    #     print("height",height)

    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rospy.logwarn("Could not get transform from 'base_link' to "+str(candy_frame_id))
    #     return
    move_base_distance,height,wrist_extension = self.scan_and_get_aruco(candy_frame_id)
    # sending move base command
    # move_base_traj_point = [move_base_distance]
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(0.0)
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.goal_time_tolerance = rospy.Time(1.0)

    
    joint_name = 'translate_mobile_base'
    trajectory_goal.trajectory.joint_names = [joint_name]         
    

    point.positions = [move_base_distance]

    trajectory_goal.trajectory.points = [point]
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()
    self.trajectory_client.send_goal(trajectory_goal)
    self.trajectory_client.wait_for_result()
    # trajectory_goal.trajectory.header.frame_id = 'base_link'
    # self.trajectory_client.send_goal(trajectory_goal)

    # Initialize the positions we want to reach to grasp candy
    # reach_pre_grasp = [0.20268437786715443, 0.9285284092581446, 0.25, 0.1674595693441825, -1.8338199879850292, 0.0]
    reach_pre_grasp = [0.20268437786715443, self.wrap_height(height+0.15), self.wrap_extension(wrist_extension-0.05), self.wrist_yaw_stow, self.head_pan, self.head_tilt]
    
    #lower_arm_pre_grasp = [0.20268437786715443, 0.7513784372841598, 0.3, 0.1674595693441825, -1.8338199879850292, 0.0]
    lower_arm_pre_grasp = [0.20268437786715443, self.wrap_height(height+0.15), self.wrap_extension(wrist_extension), self.wrist_straight, self.head_pan, self.head_tilt]
    lower_arm_pre_grasp2 = [0.20268437786715443, self.wrap_height(height+0.05), self.wrap_extension(wrist_extension), self.wrist_straight, self.head_pan+0.5, self.head_tilt]
    
    grasp_candy= [-0.12637118506165643, self.wrap_height(height), self.wrap_extension(wrist_extension), self.wrist_straight, self.head_pan, self.head_tilt]

    # Creation of the trajectory point data structure
    trajectory_points = [
        self.create_trajectory_point(reach_pre_grasp),
        self.create_trajectory_point(lower_arm_pre_grasp),
        self.create_trajectory_point(lower_arm_pre_grasp2),
        self.create_trajectory_point(grasp_candy)
    ]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = self.joint_names

    for point in trajectory_points:
        trajectory_goal.trajectory.points.append(point)

    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    #rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    rospy.loginfo('Sent trajectory')
    rospy.loginfo('Waiting for result ....')
    self.trajectory_client.wait_for_result()
    rospy.loginfo('Got results!')
    time.sleep(1)
    self.reach_candy_delivery_pose()
    time.sleep(2)
    self.stow_for_visibility()


  def pick_and_place_candy(self):
    """
    Function that initiates the multipoint_command function.
    :param self: The self reference.

    This will execute a single pick and place operation.
    """
    hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
    rospy.loginfo('issuing grasp candy command...')
    self.grasp_candy_command()
    time.sleep(3)
    rospy.loginfo('issuing release candy command...')
    self.reach_candy_delivery_pose()
    time.sleep(2)
  def main(self):
    """
    Function that initiates the multipoint_command function.
    :param self: The self reference.
    """
    hm.HelloNode.main(self, 'ArucoCandyDispenser', 'ArucoCandyDispenser', wait_for_first_pointcloud=False)
    # rospy.loginfo('issuing multipoint command...')
    # self.issue_multipoint_command()
    # time.sleep(2)



def main():
  try:
    print("Starting ArucoGraspCandy")
    node = ArucoGraspCandy()
    node.main()
    # print("sending command")
    # node.aruco_grasp_candy_command(3)
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')

if __name__ == '__main__':
  main()