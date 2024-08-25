# goat
Read Me


https://a360.co/3Abcq31

              
1) a) Workspace - catkin_ws
   b) package   - catkin_ws/src/bot_description

2)  Mechanical Design is done by using Fusion 360 and converted into ROS by plugin https://github.com/syuntoku14/fusion2urdf  

3) To Visualize the Rviz
    roslaunch bot_description rviz.launch
    
4) To Visualize in Gazebo
    roslaunch bot_description spawn.launch

5) For differential Drive plugin and teleop for moving the robot
   roslaunch bot_description control.launch 
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py


2)  pakage - bot_world
		-worlds
		-launch
		
   To launch the gazebo World ( not same in the question)
		- roslaunch bot_world gazebo_world.launch
			

 
  roslaunch bot_description gazebo.launch
  roslaunch bot_description naviation.launch 


for Moving tables
run goal.py
this scripts run all points and ends in home position


for  giving order for seperate tables
 run robot_navigator.py
 order.py
load waypints

rosparam load waypoints.yaml





