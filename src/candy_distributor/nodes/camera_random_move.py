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