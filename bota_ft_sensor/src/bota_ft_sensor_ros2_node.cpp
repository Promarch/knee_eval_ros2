// bota_ft_sensor_ros2_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

// System headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <string>
#include <memory>
#include <vector>
#include <numeric>

// Include the Bota FT sensor communication class
#include "BotaForceTorqueSensorComm.h"

class BotaFTSensorNode : public rclcpp::Node, public BotaForceTorqueSensorComm
{
public:
  BotaFTSensorNode() : Node("bota_ft_sensor_node")
  {
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("frame_id", "ft_sensor_link");
    this->declare_parameter("publish_intervall", 10);  // ms
    this->declare_parameter("zero_samples", 100);
    this->declare_parameter("zero_timeout_ms", 5000);

    // Get parameters
    serial_port_name_ = this->get_parameter("serial_port").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_intervall_ = this->get_parameter("publish_intervall").as_int();
    zero_samples_ = this->get_parameter("zero_samples").as_int();
    zero_timeout_ms_ = this->get_parameter("zero_timeout_ms").as_int(); 

    // Create publishers
    wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("ft_sensor/wrench", 10);
    wrench_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("ft_sensor/wrench_stamped", 10);

    // Create zeroing service
    zero_service_ = this->create_service<std_srvs::srv::Trigger>(
      "ft_sensor/zero",
      std::bind(&BotaFTSensorNode::zeroSensorCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Reset zero service
    reset_zero_service_ = this->create_service<std_srvs::srv::Trigger>(
      "ft_sensor/reset_zero",
      std::bind(&BotaFTSensorNode::resetZeroCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    // Initialize the serial port
    if (!initializeSerialPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port. Exiting.");
      return;
    }
    
    // Create timer for reading sensor data
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_intervall_),
      std::bind(&BotaFTSensorNode::readAndPublish, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Bota Force Torque Sensor node initialized on port %s", serial_port_name_.c_str());
  }

  ~BotaFTSensorNode()
  {
    if (serial_port_ >= 0) {
      close(serial_port_);
      RCLCPP_INFO(this->get_logger(), "Closed serial port");
    }
  }

private:

  // Declare Member variables
  std::string serial_port_name_;
  std::string frame_id_;
  int publish_intervall_; 
  int serial_port_ = -1;
  // Declare publisher
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_stamped_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Variables related to zeroing the sensor
  bool is_zeroed_;
  int zero_samples_;
  int zero_timeout_ms_;
  std::array<float, 6> zero_force_offset_; 
  std::vector<std::array<float, 6>> zero_force_samples_; 

  // Declare zeroing services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_zero_service_; 

  // Function to zero out the sensor
  void zeroSensorCallback(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    // Clear previous samples
    zero_force_samples_.clear();

    // Collect samples
    auto start_time = this->now();
    while (zero_force_samples_.size() < static_cast<size_t>(zero_samples_)) 
    {
      // Check timeout
      if ((this->now() - start_time).seconds() > (zero_timeout_ms_/1000)) {
        response->success = false; 
        response->message = "Zeroing timeout. Not enough valid samples."; 
        return; 
      }

      // Read a frame
      switch (readFrame())
      {
        case VALID_FRAME:
          if (frame.data.status.val == 0) {
            std::array<float, 6> current_forces = {
              frame.data.forces[0], 
              frame.data.forces[1], 
              frame.data.forces[2], 
              frame.data.forces[3], 
              frame.data.forces[4], 
              frame.data.forces[5], 
            };
            zero_force_samples_.push_back(current_forces); 
          }
          break; 

        case NOT_VALID_FRAME:
        case NOT_ALLIGNED_FRAME:
        case NO_FRAME:
          break; 
      }
    }

    // Sum the measured values
    std::array<float, 6> total_forces = {0,0,0,0,0,0};
    for (const auto& sample : zero_force_samples_) {
      for (int i=0;i<6;i++) {
        total_forces[i] += sample[i];
      }
    }

    // Compute the average offsets
    for (int i=0; i<6; i++) {
      zero_force_offset_[i] = total_forces[i] / zero_force_samples_.size(); 
    }

    // Mark as zeroed
    is_zeroed_ = true; 

    // Prepare response
    response->success = true; 
    response->message = "Sensor succesfully zeroed. ";

    // Log the zeroing values
    RCLCPP_INFO(this->get_logger(), "Zeroing offsets: ");
    for (int i=0; i<6; i++) {
      RCLCPP_INFO(this->get_logger(), "Offset %d: %f", i, zero_force_offset_[i]);
    }
  }

  void resetZeroCallback (const std_srvs::srv::Trigger::Request::SharedPtr /*request*/, std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    // Reset zeroing
    is_zeroed_ = false;
    zero_force_offset_ = {0,0,0,0,0,0};
    zero_force_samples_.clear();
    
    response->success = true;
    response->message = "Sensor zeroing reset.";
    RCLCPP_INFO(this->get_logger(), "Zeroing values reset. ");
  }

  bool initializeSerialPort()
  {
    serial_port_ = open(serial_port_name_.c_str(), O_RDWR);
    
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i from opening device: %s", 
                  errno, strerror(errno));
      if (errno == 13) {
        RCLCPP_ERROR(this->get_logger(), "Add the current user to the dialout group");
      }
      return false;
    }

    // Create new termios struct
    struct termios tty;
    struct serial_struct ser_info;
    memset(&tty, 0, sizeof(tty));

    // Read in existing settings, and handle any error
    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
      return false;
    }

    tty.c_cflag &= ~PARENB;  // Disable parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag |= CS8;      // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;  // Disable canonical mode
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes
    tty.c_oflag &= ~OPOST;   // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;   // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds)
    tty.c_cc[VMIN] = 0;

    // Set baud rate to 460800
    cfsetispeed(&tty, B460800);
    cfsetospeed(&tty, B460800);

    // Save tty settings
    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
      return false;
    }

    // Enable linux FTDI low latency mode
    ioctl(serial_port_, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(serial_port_, TIOCSSERIAL, &ser_info);

    return true;
  }

  void readAndPublish()
  {
    switch (readFrame())
    {
      case VALID_FRAME:
        if (frame.data.status.val > 0)
        {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                              "No valid forces: app_took_too_long=%i, overrange=%i, invalid_measurements=%i, raw_measurements=%i",
                              frame.data.status.app_took_too_long,
                              frame.data.status.overrange,
                              frame.data.status.invalid_measurements,
                              frame.data.status.raw_measurements);
        }
        else
        {
          publishWrenchData();
        }
        break;
      
      case NOT_VALID_FRAME:
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "No valid frame: CRC error count %i", get_crc_count());
        break;
      
      case NOT_ALLIGNED_FRAME:
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Lost sync, trying to reconnect");
        break;
      
      case NO_FRAME:
        // No data available, nothing to do
        break;
    }
  }

  void publishWrenchData()
  {
    auto wrench_msg = std::make_unique<geometry_msgs::msg::Wrench>();
    auto wrench_stamped_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
    
    // Apply zeroing if enabled
    std::array<float, 6> applied_forces; 
    // look up ternary operators for if-else to understand the following syntax
    for (int i=0;i<6;i++) {
      applied_forces[i] = (is_zeroed_) ? frame.data.forces[i] - zero_force_offset_[i] : frame.data.forces[i];
    }
    // Populating wrench message, assuming the forces array contains [Fx, Fy, Fz, Tx, Ty, Tz]
    wrench_msg->force.x = applied_forces[0];
    wrench_msg->force.y = applied_forces[1];
    wrench_msg->force.z = applied_forces[2];
    wrench_msg->torque.x = applied_forces[3];
    wrench_msg->torque.y = applied_forces[4];
    wrench_msg->torque.z = applied_forces[5];
    
    // Set up the WrenchStamped message
    wrench_stamped_msg->header.stamp = this->now();
    wrench_stamped_msg->header.frame_id = frame_id_;
    wrench_stamped_msg->wrench = *wrench_msg;
    
    // Publish the messages
    wrench_publisher_->publish(*wrench_msg);
    wrench_stamped_publisher_->publish(*wrench_stamped_msg);
    
    // Optional: publish sensor temperature or other diagnostics
    RCLCPP_DEBUG(this->get_logger(), 
                "Sensor temperature: %.2f°C, Timestamp: %u", 
                frame.data.temperature, 
                frame.data.timestamp);
  }

  // Implement the pure virtual methods from BotaForceTorqueSensorComm
  int serialReadBytes(uint8_t* data, size_t len) override {
    return read(serial_port_, data, len);
  }
  
  int serialAvailable() override {
    int bytes;
    ioctl(serial_port_, FIONREAD, &bytes);
    return bytes;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BotaFTSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
