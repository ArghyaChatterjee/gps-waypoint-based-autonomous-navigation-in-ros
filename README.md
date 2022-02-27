# GPS-waypoint-based-Autonomous-Navigation-in-ROS
GPS points will be predefined for the robot to navigate to the destination avoiding obstacles.

This repo package was tested on a Custom Rover with Razor 9DOF IMU, ZED F9P (RTK2) GPS, and RPLidar A1 lidar. The base station laptop is running Ubuntu 16.04 and the rover is running Ubuntu 16.04 on an Nvidia Jetson TX2. 

## Run the package

In your terminal, navigate to your catkin_ws's source (src) directory & run:
```
cd catkin_ws/src
git clone https://github.com/ArghyaChatterjee/gps-waypoint-based-autonomous-navigation-in-ros.git
cd ..
catkin_make
```
In that terminal, launch the navigation file:
```
source devel/setup.bash
roslaunch gps_waypoint_nav gps_waypoint_nav.launch
```
In another terminal, launch the joystick controller file:
```
source devel/setup.bash
roslaunch gps_waypoint_nav joy_launch_control.launch
```
Run the rover with the joystick. During the run, press "LB" to start collecting waypoints. The waypoints will be saved inside 'points_outdoor.txt'. When the run is finished, press "RB" to start following waypoints. 

<p align="center">
    <img src="assets/gps_image.png", width="800">
</p>

## User Interface Demo
### Mapviz package
We have used mapviz package to visualize the path and the cordinates. 

#### Binary Install:
```
sudo apt-get install ros-kinetic-mapviz \
                       ros-kinetic-mapviz-plugins \
                       ros-kinetic-tile-map \
                       ros-kinetic-multires-image		       
```
#### Source Install:
```
cd catkin_ws/src
git clone https://github.com/swri-robotics/mapviz.git
rosdep install --from-paths src --ignore-src
catkin_make
```
Delete any previous configuration file that you have worked with. The sequence of plugins in your panel is vital as Mapviz draws its plugins in the order that they are listed in the plugin panel. If navsat is listed first, it will draw that first, and then it will draw the tile_map over that, so you would not be able to see any fixes.
```
sudo rm ~/.mapviz_config
```
In one terminal run:
```
roscore
```
In another terminal, launch the mapviz file. Your `mapviz.launch` file should look like this:
```
<launch>

  <node pkg="mapviz" type="mapviz" name="mapviz"></node>

  <node pkg="swri_transform_util" type="initialize_origin.py" name="initialize_origin" >
    <param name="local_xy_frame" value="/map"/>
    <param name="local_xy_origin" value="auto"/>
    <!--<param name="local_xy_origin" value="swri"/>-->
    <rosparam param="local_xy_origins">
      [{ name: swri,
         latitude: 29.45196669,
         longitude: -98.61370577,
         altitude: 233.719,
         heading: 0.0},

       { name: back_40,
         latitude: 29.447507,
         longitude: -98.629367,
         altitude: 200.0,
         heading: 0.0}]
    </rosparam>
    <remap from="fix" to="/navsat/fix"/>
  </node>

  <node pkg="tf" type="static_transform_publisher" name="swri_transform" args="0 0 0 0 0 0 /map /origin 100"  />

</launch>
```
Use the following command to launch the `mapviz.launch` file:
```
cd catkin_ws
source devel/setup.bash
roslaunch mapviz mapviz.launch
```
In the panel, leave the 1st box (fixed frame "Map" and target frame "None") as it is, add tile_map and then add navsat plugin (select topic /navsat/fix). Perform it sequencially. 

#### Mapviz Testing:
Now, you need a sample bag file which will publish the gps in `/navsat/fix` topic. Download the rosbag from [here](https://advdataset2019.wixsite.com/urbanloco/hong-kong). Run the rosbag in another terminal:
```
rosbag play test2.bag
```
<p align="center">
    <img src="assets/mapviz_satellite.gif", width="800">
</p>

### Rviz Satellite Package
#### Source Install:
```
cd catkin_ws/src
git clone https://github.com/nobleo/rviz_satellite.git
catkin_make
```
Use the following command to launch the `rviz.launch` file:
```
cd catkin_ws
source devel/setup.bash
roslaunch rviz_satellite demo.launch
```
#### Rviz Satellite Testing:
You need a sample bag file which will publish the gps in `/navsat/fix` topic. Download the rosbag from [here](https://advdataset2019.wixsite.com/urbanloco/hong-kong). Run the rosbag in another terminal:
```
rosbag play CA-20190828184706_blur_align.bag
```
<p align="center">
    <img src="assets/rviz_satellite.gif", width="800">
</p>
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
  3. https://github.com/swri-robotics
  4. https://github.com/danielsnider/follow_waypoints
  5. https://github.com/ros-geographic-info
  6. https://github.com/nobleo/rviz_satellite
