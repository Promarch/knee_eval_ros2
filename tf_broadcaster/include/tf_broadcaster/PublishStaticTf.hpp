#ifndef TF_BROADCASTER_ADD_STATIC_TF_HPP_
#define TF_BROADCASTER_ADD_STATIC_TF_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"

class PublishStaticTf : public rclcpp::Node {
public: 
    PublishStaticTf(); 

private: 
    // Declare functions
    void publish_tf(const std::vector<double>& tf_RPY, const std::string& world_frame, const std::string& child_frame);
    std::vector<double> parse_double_array(const std::string& str);

    // Declare members
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    std::vector<double> SensorInTibia_RPY_;
    std::vector<double> KneeInTibia_RPY_;

    std::string sensor_param_; 
    std::string knee_param_; 
};

#endif // TF_BROADCASTER_ADD_STATIC_TF_HPP_