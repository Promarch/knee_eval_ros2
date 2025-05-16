#include "tf_broadcaster/PublishDebug.hpp"

PublishDebug::PublishDebug() : Node("PublishStaticTf") {

    // Declare and get parameters
    this->declare_parameter("femur_distance", "");
    this->declare_parameter("tibia_distance", "");

    // Input is a list of 6 double, of the form [x, y, z, roll, pitch, yaw]
    femur_distance_str_ = this->get_parameter("femur_distance").as_string(); 
    tibia_distance_str_ = this->get_parameter("tibia_distance").as_string(); 

    // Transform string to vector
    femur_distance_ = parse_double_array(femur_distance_str_);
    tibia_distance_ = parse_double_array(tibia_distance_str_);

    // Initialize Broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this); 

    // Publish transforms
    publish_tf(femur_distance_, "femur_ref", "femur_contact");
    publish_tf(tibia_distance_, "tibia_ref", "tibia_contact");

    RCLCPP_INFO(this->get_logger(), "Finished publishing contact points of femur and tibia");

}

std::vector<double> PublishDebug::parse_double_array(const std::string& str) {
    std::vector<double> result;
    std::stringstream ss(str);
    std::string item;
    
    while (std::getline(ss, item, ',')) {
        try {
            result.push_back(std::stod(item));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse value: %s", item.c_str());
        }
    }
    
    return result;
}

void PublishDebug::publish_tf(const std::vector<double>& tf_RPY, 
                                const std::string& world_frame, const std::string& child_frame) {

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now(); 
    t.header.frame_id = world_frame;
    t.child_frame_id = child_frame;

    // Copy translation from launch argument
    t.transform.translation.x = tf_RPY[0];
    t.transform.translation.y = tf_RPY[1];
    t.transform.translation.z = tf_RPY[2];

    // Copy rotation and transform to quaternion
    tf2::Quaternion q; 
    q.setRPY(
        tf_RPY[3], 
        tf_RPY[4], 
        tf_RPY[5]);
    t.transform.rotation.x = q.x(); 
    t.transform.rotation.y = q.y(); 
    t.transform.rotation.z = q.z(); 
    t.transform.rotation.w = q.w(); 

    tf_static_broadcaster_->sendTransform(t); 
}

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishDebug>());
    rclcpp::shutdown(); 
    
    return 0;
}