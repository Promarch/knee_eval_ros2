# Bota Force Torque Sensor ROS2 Node

## Features
- Publishes force/torque data as ROS2 Wrench messages
- Provides services for zeroing the sensor
- Configurable via ROS2 parameters

## Topics
- `/ft_sensor/wrench`: Raw force/torque data
- `/ft_sensor/wrench_stamped`: Force/torque data with timestamp

## Services
- `/ft_sensor/zero`: Zero out the current sensor readings
  - Collects a specified number of samples
  - Calculates and applies an offset to future readings
- `/ft_sensor/reset_zero`: Reset the zeroing, removing any applied offsets

## Parameters
- `serial_port` (string, default: "/dev/ttyUSB0"): Serial port for the sensor
- `frame_id` (string, default: "ft_sensor_link"): TF frame ID for the measurements
- `publish_rate` (double, default: 100.0): Publishing rate in Hz
- `zero_samples` (int, default: 100): Number of samples to average for zeroing
- `zero_timeout_ms` (int, default: 5000): Timeout for zeroing in milliseconds

## Usage Examples

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

### Subscribe to Topics
```bash
# Listen to raw wrench data
ros2 topic echo /ft_sensor/wrench

# Listen to stamped wrench data
ros2 topic echo /ft_sensor/wrench_stamped
```

## Troubleshooting
- Ensure you have the correct serial port specified
- Add your user to the dialout group if you encounter permission issues
  ```bash
  sudo usermod -a -G dialout $USER
  ```
