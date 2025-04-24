#include "calc_force_knee/calc_force_knee.hpp"

CalcForceKnee::CalcForceKnee() : Node("CalcForceKnee") {

    // Declare and get parameters
    this->declare_parameter("target_frame", "knee_ref");
    this->declare_parameter("source_frame", "sensor_ref");
    target_frame_ = this->get_parameter("target_frame").as_string(); 
    source_frame_ = this->get_parameter("source_frame").as_string();     

    // Initialize frame listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

    // Initialize force subscriber
    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/ft_sensor/wrench_stamped", 10, 
        std::bind(&CalcForceKnee::ForceCallback, this, std::placeholders::_1));

    // Initialize transformed force publisher
    force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/wrench_knee", 10);    
}

void CalcForceKnee::ForceCallback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {

    // Get the latest transformation
    geometry_msgs::msg::TransformStamped tf_stamped; 
    try {
        tf_stamped = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero); // the function has permanent access to member variables
    }
    catch (const tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s; %s", source_frame_.c_str(), target_frame_.c_str(), e.what());
        return; 
    }

    // Calculate the force/torque in the new frame
    auto wrench_transformed = TransformWrench(msg->wrench, tf_stamped.transform);

    // Create output message
    geometry_msgs::msg::WrenchStamped wrench_out;  
    wrench_out.header.stamp = this->now(); 
    wrench_out.header.frame_id = target_frame_; 
    wrench_out.wrench = wrench_transformed;

    force_pub_->publish(wrench_out); 
}

geometry_msgs::msg::Wrench CalcForceKnee::TransformWrench(const geometry_msgs::msg::Wrench& wrench_in, 
                                                            const geometry_msgs::msg::Transform& tf) {

    // Extract rotation and translation from tf
    tf2::Quaternion rotation(
        tf.rotation.x, 
        tf.rotation.y, 
        tf.rotation.z, 
        tf.rotation.w); 
    tf2::Vector3 translation(
        tf.translation.x, 
        tf.translation.y, 
        tf.translation.z); 
    // Extract wrench components
    tf2::Vector3 force_in(wrench_in.force.x, wrench_in.force.y, wrench_in.force.z);
    tf2::Vector3 torque_in(wrench_in.torque.x, wrench_in.torque.y, wrench_in.torque.z);

    // Transform force
    tf2::Vector3 force_out = tf2::quatRotate(rotation, force_in); 
    // Transform torque (also consider lever arm)
    tf2::Vector3 torque_rotated = tf2::quatRotate(rotation, torque_in);
    tf2::Vector3 torque_from_force = translation.cross(force_in); 
    tf2::Vector3 torque_out = torque_rotated + torque_from_force; 
    
    // Create output wrench
    geometry_msgs::msg::Wrench wrench_rotated; 
    wrench_rotated.force.x = force_out[0]; 
    wrench_rotated.force.y = force_out[1]; 
    wrench_rotated.force.z = force_out[2]; 
    wrench_rotated.torque.x = torque_out[0]; 
    wrench_rotated.torque.y = torque_out[1]; 
    wrench_rotated.torque.z = torque_out[2]; 
    
    return wrench_rotated;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<CalcForceKnee>()); 
    rclcpp::shutdown(); 
    return 0;
}