# Packages for knee evaluation

This repo contains all packages necessary to run the knee evaluation setup. It contains the following packages

### mocap4ros_optitrack & mocap_msgs

[Packages published by OptiTrack](https://github.com/OptiTrack/mocap4ros2_optitrack) to stream rigid bodies, markers or skeletons to ROS2. 

### bota_ft_sensor

Streams Force-Torque data from the sensor. Adapted from the [official driver](https://gitlab.com/botasys/bota_serial_driver) provided by BotaSystems. 

### tf_broadcaster

Contains two separate notes: AddFrameSensor broadcasts the static transformation between the optitrack frame and the marker frame, while AddStaticTf broadcasts the static transformation between the frame of the sensor in relation to the tibia tracker as well as the transformation between the knee joint and the tibia tracker. 

### calc_force_knee

This one contains also contains two nodes, though one is optional. The main node is called CalcForceKnee, which subscribes to the sensor data topic, applies the transformation from the sensor frame to the knee frame, and publishes the newly calculated sensor data in the knee frame. The other node is called ForceVisualizer which subscribes to the force published by CalcForceKnee and publishes an arrow marker for RViz. 

### csv_writer

As the name says, this node writes a csv file comprising the calculated force in the knee as well as the frames of the tibia tracker, the femur tracker and the position of the knee joint. The frame of the knee joint has the same orientation as the tibia tracker. 

## Build

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
For ease of use, add the newly created repository to .bashrc so that you don't have to source the environment every time. Please replace "knee_eval_ws" with your chosen workspace name. 
```
echo "~/knee_eval_ws/install/setup.bash" >> ~/.bashrc
```
## Usage
### General Node Setup
All packages needed for the evaluation are launched via the main launch file. This launches the OptiTrack node with the static transformations, the force sensor with the force transformation node, as well as rviz to visualize the transformation. Rviz is not necessary for the final data stream and can be closed as soon as it is launched. Navigate into the root directory of your workspace and type: 
```
ros2 launch pkg_launcher global_launcher.launch.py
```
### CSV Writer
The node to write the CSV file is started separately. Currently, the node starts writing the file as soon as it is launched and receives transformation, and closes the file when the node is destroyed. Trigger based writing and closing will be implemented in a later iteration. 
```
ros2 launch csv_writer csv_writer.launch.py
```

