# Packages for knee evaluation

This repo contains all packages necessary to run the knee evaluation setup. It contains the following packages

### mocap4ros_optitrack & mocap_msgs

[Packages published by OptiTrack](https://github.com/OptiTrack/mocap4ros2_optitrack) to stream rigid bodies, markers or skeletons to ROS2. 

### bota_ft_sensor

Streams Force-Torque data from the sensor. Adapted from the [official driver](https://gitlab.com/botasys/bota_serial_driver) provided by BotaSystems. 

### tf_broadcaster

Broadcasts the static transformation between the different coordinate systems, i.e. from the OptiTrack CoSy to the tibia CoSy. 

## Usage

Create a workspace
```
mkdir -p knee_eval_ws/src && cd knee_eval_ws/src
```
Copy the content of the repository in the folder
```
git clone https://github.com/Promarch/knee_eval_ros2.git .
```
Check for missing dependencies (in the root of your workspace) before building: 
```
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```
Build the packages
```
colcon build --packages-select tf_broadcaster bota_ft_sensor mocap_optitrack_driver
```
The nodes need to be launched individually at the moment, see the package descriptions for instructions. 
