# tf_broadcaster

This package contains two nodes, both publishing static transformations. 
AddFrameSensor broadcasts the static transformation between the optitrack frame and the marker frame, while AddStaticTf broadcasts the static transformation between the frame of the sensor in relation to the tibia tracker as well as the transformation between the knee joint and the tibia tracker. 

### General Usage

Both nodes are started by the same launch file
```
ros2 launch tf_broadcaster add_frame_sensor.launch.py
```
Running the node directly via `ros2 run ...` is possible, though you have to set the parameters manually since there are no default parameters at the moment. 

## Node description

### AddFrameSensor

This node uses the kabsch algorithm to calculate the transformation from the optitrack coordinate system to the coordinate system of the tracker. The reference and optitrack configuration is saved in the config/marker_coordinates.yaml file, which is in turn generated via the python file convert_tracker_csv.py. The original optitrack configuration is directly exported from motive, and the tracker configuration is generated from the STL file. 

#### Parameters

- `marker_file_path`: File path to the yaml file where the marker coordinates are stored. 

### AddStaticTf

This node simply generates the static transformation from the measured distance between tibia tracker and sensor/knee joint. The distance can be changed via the parameters/the launch file. 

#### Parameters

- `SensorInTibia_RPY`: Describes the transformation necessary to go from the tibia to the sensor frame. The individual values are: x,y,z,roll, pitch, yaw. 
- `KneeInTibia_RPY`: Identical to the parameter above, but describes the transformation from the tibia to the knee. 