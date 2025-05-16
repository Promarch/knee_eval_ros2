#ifndef TF_BROADCASTER_PUBLISH_DEBUG_HPP_
#define TF_BROADCASTER_PUBLISH_DEBUG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"

class PublishDebug : public rclcpp::Node {
public: 
    PublishDebug(); 

private: 
    // Declare functions
    void publish_tf(const std::vector<double>& tf_RPY, const std::string& world_frame, const std::string& child_frame);
    std::vector<double> parse_double_array(const std::string& str);

    // Declare members
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    // Imported params
    std::string femur_distance_str_;
    std::string tibia_distance_str_;

    // Store raw values
    std::vector<double> femur_distance_; 
    std::vector<double> tibia_distance_; 

};

#endif // TF_BROADCASTER_PUBLISH_DEBUG_HPP_