#ifndef CALC_FORCE_KNEE_HPP_
#define CALC_FORCE_KNEE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_msgs/msg/tf_message.hpp"


class CalcForceKnee : public rclcpp::Node {
public: 
    CalcForceKnee(); 

private: 
    // Declare functions
    void ForceCallback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg_force); 
    geometry_msgs::msg::Wrench TransformWrench(const geometry_msgs::msg::Wrench& wrench_in, 
                                                const geometry_msgs::msg::Transform& tf); 

    // Declare members
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::string target_frame_;
    std::string source_frame_;
};

#endif //CALC_FORCE_KNEE_HPP_