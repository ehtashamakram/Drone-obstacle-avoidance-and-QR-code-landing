This is a ROS package of AR. Drone: from an initial robot configuration.

To run the package: 

- Extract the package to the catkin workspace.

- Start by adding two repositories using rVis to initiate the Panda robot.

- Source your catkin workspace, Open terminal window and commands: 
  $ cd catkin_ws
  $ catkin_make
  $ source devel/setup.bash 


-1 Launch Ar. Drone in Gazebo and rviz using the following command:
  $ roslaunch drone_gazebo drone.launch

-2 Enable the drone motors using command:
  $ rosservice call /enable_motors "enable: true"

-2 Run the following commands using a separate terminal:
  $ rosrun drone_control takeoff.py

- Wait to adjust hieght above 1.3 and below 1.5

-3 Run the following command in another terminal:
  $ roslaunch drone_navigation move_base.launch 
  


-4 Run the following command in another terminal:
  $ rosrun drone_control goal.py

- wait until calibration completed 400

-5 Run the following command in another terminal:
  $ rosrun drone_control land.py

