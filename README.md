# stretchit_halloween_candy_distributer

This is a ROS1 workspace built to distribute candies(technically it can be anything that Stretch can grasp) using Stretch.

It uses the Xbox controller to trigger the pick and place candy task.

The action is hardcoded and the starting and end postion is static for now.

### How to run it ?
- Build the workspace
    - `cd stretchit_halloween_candy_distributer`
    - `catkin_make`
- Run the Nodes
    - `stretch_robot_home.py`
    - `roslaunch stretch_core stretch_driver.launch`
    - `roslaunch candy_distributor candy_distributor.launch`

### Features
- Continuous mode - Distributes the candies continuously.
- Discrete mode - Distributes the candy only once.
- Special sound effects 

### What is there is no xboz controller, will this repository still work?
- the modes can be switched using an xbox controller or by publishing the below strings on `joystick_state` topic.
    - "A" - Run pick and place action once
    - "B" - Toggle the continuous pick and place action

### dependencies
- pip
    - `Google text to speech -gTTS`
    - `playsound`
- other
    - [stretch_ros workspace](https://github.com/hello-robot/stretch_ros)
    - [robot-upstart](http://docs.ros.org/en/jade/api/robot_upstart/html/)

### StartUp
- add the launch file to startup
    - `rosrun robot_upstart install candy_distributor/launch/candy_distributor.launch`
- to stop the process after autostart (for whatever reason)
    - `sudo systemctl stop candy`


