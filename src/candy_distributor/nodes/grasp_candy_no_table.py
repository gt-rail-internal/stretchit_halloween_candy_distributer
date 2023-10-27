#!/usr/bin/env python3

import rospy
import time
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from xbox_ros.msg import joystick
from std_msgs.msg import String
from datetime import datetime, timedelta
import threading
import random

class GraspCandy(hm.HelloNode):
  def __init__(self):
    #hm.HelloNode.__init__(self)
    # Declare joint names of all Stretch Joints
    hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
    self.joint_names = [
        'joint_gripper_finger_left',  # Gripper only has 1DOF therefore we only choose one of the following options: 'joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture'
        'joint_lift',
        'wrist_extension',
        'joint_wrist_yaw',
        'joint_head_pan',
        'joint_head_tilt'
    ]
    self.give_candy_loop = False
    self.give_candy_once = False
    self.last_chirp_at = None
    self.last_looked_around_at = None
    self.elevation_factor = 0

  def create_trajectory_point(self, positions):
    point = JointTrajectoryPoint()
    point.positions = positions
    return point
  
  def reach_candy_delivery_pose(self):
    # Initialize the positions we want to reach to grasp candy
    lift_arm_grasp = [-0.12637118506165643, 0.8705166073539916, 0.0999928970260517, 3.779345166153249, -1.8338199879850292, 0.0]
    reach_delivery = [-0.12637118506165643, 0.8705166073539916, 0.33989182966440784, -0.01086569724752329, -1.8338199879850292, 0.0]
    release_grasp = [0.20268437786715443,  0.8705166073539916, 0.33989182966440784, -0.01086569724752329, -1.8338199879850292, 0.0]
    return_to_pregrasp = [0.20268437786715443, 0.8705166073539916, 0.33989182966440784, -0.01086569724752329, -1.8338199879850292, 0.0]

    # Creation of the trajectory point data structure
    reach_trajectory_points = [
        self.create_trajectory_point(lift_arm_grasp),
        self.create_trajectory_point(reach_delivery),
        
    ]

    

    release_trajectory_points = [
      self.create_trajectory_point(release_grasp)
    ]

    return_to_pregrasp_points = [self.create_trajectory_point(return_to_pregrasp)]


    # Reach delivery
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = self.joint_names

    for point in reach_trajectory_points:
        trajectory_goal.trajectory.points.append(point)
   
    

    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    #rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

    t = threading.Thread(name='child procs', target=self.beep)
    t.start()

    # Release Candy
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = self.joint_names
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'
    for point in release_trajectory_points:
        trajectory_goal.trajectory.points.append(point)
    
    self.trajectory_client.send_goal(trajectory_goal)
    #rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

    #Return to pregrasp
    time.sleep(2)
    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = self.joint_names
    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'
    for point in return_to_pregrasp_points:
        trajectory_goal.trajectory.points.append(point)
    
    self.trajectory_client.send_goal(trajectory_goal)
    #rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def grasp_candy_command(self):
    '''
    self.joint_names = [
        'joint_gripper_finger_left',  # Gripper only has 1DOF therefore we only choose one of the following options: 'joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture'
        'joint_lift',
        'wrist_extension',
        'joint_wrist_yaw',
        'joint_head_pan',
        'joint_head_tilt'
    ]
    '''

    # Initialize the positions we want to reach to grasp candy
    #
    reach_pre_grasp = [0.20268437786715443, 0.8705166073539916, 0.1, 0.1674595693441825, -1.8338199879850292, 0.0]
    lower_arm_pre_grasp = [0.20268437786715443, 0.2164+self.elevation_factor, 0.0999928970260517, 3.779345166153249, -1.8338199879850292, 0.0]
    grasp_candy= [-0.12637118506165643, 0.2164+self.elevation_factor, 0.0999928970260517, 3.779345166153249, -1.8338199879850292, 0.0]

    # Creation of the trajectory point data structure
    trajectory_points = [
        self.create_trajectory_point(reach_pre_grasp),
        self.create_trajectory_point(lower_arm_pre_grasp),
        self.create_trajectory_point(grasp_candy),
    ]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = self.joint_names

    for point in trajectory_points:
        trajectory_goal.trajectory.points.append(point)

    trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
    trajectory_goal.trajectory.header.frame_id = 'base_link'

    self.trajectory_client.send_goal(trajectory_goal)
    #rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
    self.trajectory_client.wait_for_result()

  def pick_and_place_candy(self):
    """
    Function that initiates the multipoint_command function.
    :param self: The self reference.

    This will execute a single pick and place operation.
    """
    
    rospy.loginfo('issuing grasp candy command...')
    
    time.sleep(2)
    self.grasp_candy_command()
    time.sleep(2)
    rospy.loginfo('issuing release candy command...')
    self.reach_candy_delivery_pose()
    time.sleep(2)
    
  
  def callback(self, data):
    B = data.right_button_pressed
    A = data.bottom_button_pressed
    D = data.bottom_pad_pressed
    U = data.top_pad_pressed
    print(data)
    if(A):
      self.give_candy_once = True

    if(B):
      self.give_candy_loop = not self.give_candy_loop
      time.sleep(0.5)
    
    if(U):
      self.elevation_factor += 0.01
    
    if(D):
      self.elevation_factor -= 0.01

  def chirp(self):
    if(self.last_chirp_at == None or datetime.now() - self.last_chirp_at>= timedelta(seconds=12)):
      self.last_chirp_at = datetime.now()
      sound_pub.publish('vader')
  
  def beep(self):
    sound_pub.publish('beep')

  def wrap_pan(self,pan):
    return max(min(pan,1.5),-4.0)
  def wrap_tilt(self,tilt):
    return max(min(tilt,0.4),-1.0)
  
  def camera_scan_non_blocking(self):
    # creating an array of pan values from -1.5 to 1.5 with random increments in between 0.1 to 1.5
    if(self.last_looked_around_at!= None and datetime.now() - self.last_looked_around_at< timedelta(seconds=6)):
      return
    self.last_looked_around_at = datetime.now()
    scan_min = -1.0
    scan_max = 1
    head_tilt = -0.2
    head_pan = -1.5
    pan_rand_weight = 0.01
    pan_rand_const = 0.005
    tilt_rand_weight = 2.0
    pan_rand_buffer_size = 10
    tilt_rand_buffer_size = 30
    head_pan_array = [head_pan+scan_min]
    random_queue = []
    for i in range(pan_rand_buffer_size):
      random_queue.append(random.random())
    for i in range(500):
      rand_no = random.random()
      random_queue.pop()
      random_queue.append(rand_no)
      rand = sum(random_queue)/len(random_queue)
      rand=rand*pan_rand_weight+pan_rand_const
      head_pan_array.append(head_pan_array[-1]+rand)
      if(head_pan_array[-1]>head_pan+scan_max):
        break
    for i in range(500):
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
    # trajectory_goal.trajectory.points.append(self.create_trajectory_point([self.wrap_pan(head_pan),self.wrap_tilt(head_tilt)]))
    
    # trajectory_goal.trajectory.points.append(self.create_trajectory_point(camera_left_pose))
    # trajectory_goal.trajectory.points.append(self.create_trajectory_point(camera_right_pose))
    self.trajectory_client.send_goal(trajectory_goal)
    return trajectory_goal

      

    

if __name__ == '__main__':
  try:
    node = GraspCandy()
    rate = rospy.Rate(60)
    rospy.Subscriber("joystick_state", joystick, node.callback, queue_size=1)
    sound_pub = rospy.Publisher('speech', String, queue_size=0)
    while(not rospy.is_shutdown()):
      if(node.give_candy_loop or node.give_candy_once):
        sound_pub.publish('intro')
        node.pick_and_place_candy()
        sound_pub.publish('outro')
        time.sleep(4)
        node.give_candy_once = False
      # Randomly rotate camera and say random sayings
      else:

        node.chirp()
        node.camera_scan_non_blocking()
        # t = threading.Thread(name='child procs', target=node.camera_scan_non_blocking())
        # t.start()
        
      rate.sleep()
  
  except KeyboardInterrupt:
    rospy.loginfo('interrupt received, so shutting down')