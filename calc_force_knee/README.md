# Calc_force_knee

ROS2 package containing nodes related to calculating and displaying the wrench in the knee frame. 
**CalcForceKnee**: necessary for the knee evaluation. This node subscribes to the wrench data published by the sensor node and the transformation between the sensor frame and the knee frame, and calculates the resulting wrench in the new coordinate system. 
**ForceVisualizer**: Publishes an arrow marker for RViz in the knee coordinate system. Run the node after the other nodes have been started, then select the marker in RViz to be displayed (add->display by topics->marker). 

## Usage

The CalcForceKnee node is launched via the launch file:
```
ros2 launch calc_force_knee calc_force_knee.launch.py
```
The ForceVisualizer node needs to be run directly
```
ros2 run calc_force_knee ForceVisualizer
```

### Parameters

#### CalcForceKnee

- `source_frame`: Frame in which the forces have been recorded. Default: "sensor_ref"
- `target_frame`: Frame in which the forces need to be calculated. Default: "knee_ref"
