#ifndef CSV_WRITER_HPP_
#define CSV_WRITER_HPP_

#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_srvs/srv/trigger.hpp"

class CsvWriter : public rclcpp::Node {
public: 
    CsvWriter(); 
    ~CsvWriter(); 

private: 
    // Declare functions
    void ForceCallback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg);
    void TimerCallback(); 
    void InitializeCsv();
    void WriteTf2Csv(const geometry_msgs::msg::TransformStamped& tf); 

    // Service Callbacks
    void HandleStartRecording(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void HandleStopRecording(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Declare members
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_; 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
    rclcpp::TimerBase::SharedPtr timer_; 

    std::ofstream csv_file_; 
    bool is_recording_; 

    // Service servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_recording_service_; 
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_recording_service_;

    // Parameters
    std::string path_csv_file_; 
    std::string source_frame_; 
    std::string tibia_frame_;
    std::string femur_frame_;
    std::string knee_frame_;
};

#endif // CSV_WRITER_HPP_