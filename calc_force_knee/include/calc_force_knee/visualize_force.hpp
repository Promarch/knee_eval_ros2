#ifndef VISUALIZE_FORCE_HPP_
#define VISUALIZE_FORCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class ForceVisualizer : public rclcpp::Node {

public: 
    ForceVisualizer(); 

private:
    // Declare functions
    void ForceCallback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg);
    visualization_msgs::msg::Marker CreateMarker(const tf2::Vector3& force);

    // Declare members
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::string target_frame_; 
    std::string source_frame_; 
};

#endif // VISUALIZE_FORCE_HPP_