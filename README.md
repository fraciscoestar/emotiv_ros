# Emotiv ROS Package

Unofficial implementation of Emotiv's mental commands in ROS for the control of simple robots.

Dependencies:
 - ROS Noetic
 - Gazebo
 - Emotiv launcher and free account

Installing and running:
 - Create a ROS working directory with catkin_make
 - Pull this repository in /catkin_ws/src/ -> git clone https://github.com/fraciscoestar/emotiv_ros
 - Create a new Cortex applicattion and insert your Cortex appication codes in /scripts/command.py
 - Compile the source code in catkin_ws -> catkin_make
 - Connect your emotiv headset and authorize the cortex app
 - Run any of the examples provided -> roslaunch emotiv_ros gazebo.launch (gazebo) OR roslaunch emotiv_ros testworld.launch (turtlesim)
