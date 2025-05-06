# csv_writer

ROS2 package containing the node CsvWriter, which subscribes to the transformed forces of calc_force_knee and the transformation of the world frame to the femur/tibia/knee frame, and writes the data to a csv file. 

## Usage

The node is started via the launch file 
```
ros2 launch csv_writer csv_writer.launch.py
```
The node start writing the csv file as soon as the node is launched and the first transformation is available, and stops writing when the node is destroyed. 

### Parameters

- `path_csv_file`: path to the csv file. Default: "data.csv"
- `source_frame`: World frame in relation to which all transformation are calculated. Default: "map"
- `tibia_frame`: Name of the tibia frame. Default: "tibia_ref"
- `femur_frame`: Name of the femur frame. Default: "femur_ref"
- `knee_frame`: Name of the knee frame. Default: "knee_ref"

