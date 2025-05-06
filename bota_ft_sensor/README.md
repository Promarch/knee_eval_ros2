# Bota Force Torque Sensor ROS2 Node

This package is a ROS2 node based on the original minimal driver published by BotaSystems ([link](https://gitlab.com/botasys/bota_serial_driver)). 

## Description

The node reads the sensor values at 100Hz and publishes it to two different topics. The sensor can be zeroed via ROS2 services (see section services). Assuming the package has been built and sourced, the node can be launched via the launch file. 

## Usage

All of the following lines need to be executed in the terminal in the root folder of your workspace. 

### Launch the Node
```bash
ros2 launch bota_ft_sensor bota_ft_sensor.launch.py
```

### Zero the Sensor
```bash
# Using ROS2 service call
ros2 service call /ft_sensor/zero std_srvs/srv/Trigger {}

# Reset zeroing
ros2 service call /ft_sensor/reset_zero std_srvs/srv/Trigger {}
```

## Topics, Parameters and services

### Topics
- `/ft_sensor/wrench`: Raw force/torque data, of message type geometry_msgs/Wrench
- `/ft_sensor/wrench_stamped`: Force/torque data with timestamp, of message type geometry_msgs/WrenchStamped

### Services
- `/ft_sensor/zero`: Zero out the current sensor readings
  - Collects a defined number of samples, specified by the parameter `zero_samples`
  - Calculates and applies an offset to future readings
- `/ft_sensor/reset_zero`: Reset the zeroing, removing any applied offsets

### Parameters
- `serial_port` (string, default: "/dev/ttyUSB0"): Serial port for the sensor
- `frame_id` (string, default: "ft_sensor_link"): Name of the frame in which the measurements are taken
- `publish_intervall` (int, default: 10): Intervall in ms between each sensor reading
- `zero_samples` (int, default: 100): Number of samples to average for zeroing
- `zero_timeout_ms` (int, default: 5000): Timeout for zeroing in milliseconds

## Troubleshooting

### Sensor connectivity
- Ensure you have the correct serial port specified
- Add your user to the dialout group if you encounter permission issues. You might have to restart your computer/accout for this to take effect. 
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- Alternatively, you can directly give yourself ownership of the device
  ```
  sudo chown <your_username> /dev/ttyUSB0
  ```

### Check if node is publishing correctly

```bash
# Listen to raw wrench data
ros2 topic echo /ft_sensor/wrench

# Listen to stamped wrench data
ros2 topic echo /ft_sensor/wrench_stamped
```