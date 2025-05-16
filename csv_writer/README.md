# csv_writer

ROS2 package containing the node CsvWriter, which subscribes to the transformed forces of calc_force_knee and the transformation of the world frame to the femur/tibia/knee frame, and writes the data to a csv file. The node is implemented as a lifecycle node to be able to deactivate it or shut it down remotely. 

## Usage

The node is started via the launch file 
```
ros2 launch csv_writer csv_writer.launch.py
```
When launched, the node idles until the command to start recording is sent. This is done via a service call: 
```
ros2 service call /start_csv_recording std_srvs/srv/Trigger {}
```
Stopping the recording is also triggered by a service call (or if the node is destroyed)
```
ros2 service call /stop_csv_recording std_srvs/srv/Trigger {}
```
If the node needs to be restarted from a different workstation, the node can to be shutdown via 
```
ros2 lifecycle set /CsvWriter shutdown
```
Restarting the node is done with the same launch file mentioned above. 
### Parameters

- `path_csv_file`: path to the csv file. Default: "data.csv"
- `source_frame`: World frame in relation to which all transformation are calculated. Default: "map"
- `tibia_frame`: Name of the tibia frame. Default: "tibia_ref"
- `femur_frame`: Name of the femur frame. Default: "femur_ref"
- `knee_frame`: Name of the knee frame. Default: "knee_ref"

