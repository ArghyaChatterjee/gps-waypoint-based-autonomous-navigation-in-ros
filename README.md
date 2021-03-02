# GPS-waypoint-based-Autonomous-Navigation-in-ROS
GPS points will be predefined in ROS based robots to navigate to the destination avoiding obstacles.
This package performs outdoor GPS waypoint navigation. It can navigate while building a map, avoiding obstacles, and can navigate continuously between each goal or stop at each goal. 

This repo is made to run on a Custom Rover with Razor 9DOF IMU, ZED F9P (RTK2) GPS, and RPLidar A1 lidar.

## Run the package

In your terminal, navigate to your catkin_ws's source (src) directory & run:
```
cd catkin_ws/src
git clone https://github.com/ArghyaChatterjee/GPS-waypoint-based-Autonomous-Navigation-in-ROS.git
cd ..
catkin_make
```
In that terminal, launch the navigation file:
```
source devel/setup.bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch
```
In another terminal, launch the joystick controller file:
```
source devel/setup.bash
roslaunch outdoor_waypoint_nav joy_launch_control.launch
```
Press "LB" to start waypoint collection.

## Package Description
This package uses a combination of the following packages:

   - ekf_localization to fuse odometry data with IMU and GPS data.
   - navsat_transform to convert GPS data to odometry and to convert latitude and longitude points to the robot's odometry coordinate system.
   - GMapping to create a map and detect obstacles.
   - move_base to navigate to the goals while avoiding obstacles 
   - goals are set using recorded or inputted waypoints.

## Node Description
The Navigation package within this repo includes the following custom nodes:
	
   - gps_waypoint to read the waypoint file, convert waypoints to points in the map frame and then send the goals to move_base.
   - gps_waypoint_continuous1 for continuous navigation between waypoints using one controller. 
   - gps_waypoint_continuous2 for continuous navigation between waypoints using another seperate controller.
   - collect_gps_waypoint to allow the user to drive the robot around and collect their own waypoints.	
   - calibrate_heading to set the heading of the robot at startup and fix issues with poor magnetometer data.
   - plot_gps_waypoints to save raw data from the GPS for plotting purposes.
   - gps_waypoint_mapping to combine waypoint navigation with Mandala Robotics' 3D mapping software for 3D mapping.

# Gratitude
  I would like to acknowledge the contribution of 2 website which helped me while making this repo.
  1. https://github.com/nickcharron
  2. https://github.com/clearpathrobotics
