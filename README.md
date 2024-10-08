# gazebo_diff_drive
Simple navigation system for a simulated differential drive vehicle in Gazebo and ROS Noetic.

# Installation
* Install ROS Noetic
* Install Gazebo Citadel
* Clone this repository into a catkin workspace
* Build it using `catkin_make` from the workspace folder

# Execution
* **Terminal 1**: export the Gazebo models to the Gazebo resource path. Adjust to your installation:

      export IGN_GAZEBO_RESOURCE_PATH="/home/catkin_ws/src/diff_drive/models/world:/home/catkin_ws/src/diff_drive/models>

  Then start the simulation (press the play button on the Gazebo gui):   

      ign gazebo world_demo.sdf

* **Terminal 2**: source ROS and the catkin workspace:

      source /opt/ros/noetic/setup.bash
      source /home/catkin_ws/devel/setup.bash

  Then launch the navigation server, Rviz, and ROS-Gazebo bridge:  

      roslaunch diff_drive demo.launch

* **Terminal 3**: configurate the navigation waypoints and send them using the navigation client. It can be used to send a single waypoint:   

      rosrun diff_drive navigation_client 3.1 2.0

  Or a file with multiple waypoints can be sent instead. See an example file for format in `config/waypoints.txt`:

      rosrun diff_drive navigation_client /home/catkin_ws/src/diff_drive/config/waypoints.txt

* **Terminal 4**: (optional) to see the progress of the navigation, subscribe to the `/Navigate/feedback` topic:

      rostopic echo /Navigate/feedback
