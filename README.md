# Packages for knee evaluation

This repo contains all packages necessary to run the knee evaluation setup, with the packages explained below. For a more comprehensive description of the individual packages please go to their respective readme files. 

### mocap4ros_optitrack & mocap_msgs

[Packages published by OptiTrack](https://github.com/OptiTrack/mocap4ros2_optitrack) to stream rigid bodies, markers or skeletons to ROS2. 

### bota_ft_sensor

Streams Force-Torque data from the sensor. Adapted from the [official driver](https://gitlab.com/botasys/bota_serial_driver) provided by BotaSystems. 

### tf_broadcaster

Contains two separate notes: PublishRefFrame broadcasts the static transformation between the optitrack frame and the marker frame, while PublishStaticTf broadcasts the static transformation between the frame of the sensor in relation to the tibia tracker as well as the transformation between the knee joint and the tibia tracker. 

### calc_force_knee

This package also contains two nodes, though one is optional. The main node is called CalcForceKnee, which subscribes to the sensor data topic, applies the transformation from the sensor frame to the knee frame, and publishes the newly calculated sensor data in the knee frame. The other node is called ForceVisualizer which subscribes to the force published by CalcForceKnee and publishes an arrow marker for RViz. 

### csv_writer

As the name says, this node writes a csv file comprising the calculated force in the knee as well as the frames of the tibia tracker, the femur tracker and the position of the knee joint. The frame of the knee joint has the same orientation as the tibia tracker. 

## Build

Create a workspace
```bash
mkdir -p knee_eval_ws/src && cd knee_eval_ws/src
```
Copy the content of the repository in the folder
```bash
git clone https://github.com/Promarch/knee_eval_ros2.git .
```
Check for missing dependencies (in the root of your workspace) before building: 
```bash
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```
Build the packages
```bash
colcon build --symlink-install
```
For ease of use, add the newly created repository to .bashrc so that you don't have to source the environment every time. Please replace "knee_eval_ws" with your chosen workspace name. 
```bash
echo "~/knee_eval_ws/install/setup.bash" >> ~/.bashrc
```

## Usage
### General Node Setup
All packages needed for the evaluation are launched via the main launch file. This launches the OptiTrack node with the static transformations, the force sensor with the force transformation node, as well as the csv_writer node. 
Navigate into the root directory of your workspace and type: 
```bash
ros2 launch pkg_launcher global_launcher.launch.py
```
The recording of the csv file is started via a service call (see the following section). 

### Service calls

It is advised to zero the sensor before the recording is started. 
```bash
ros2 service call /ft_sensor/zero std_srvs/srv/Trigger {}
```
The recording of the csv file is also controlled via service calls. 
```bash
# Start recording
ros2 service call /start_csv_recording std_srvs/srv/Trigger {}
# Stop recording
ros2 service call /stop_csv_recording std_srvs/srv/Trigger {}
```
