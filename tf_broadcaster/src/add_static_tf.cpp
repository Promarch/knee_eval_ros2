#include "tf_broadcaster/add_static_tf.hpp"

AddStaticTf::AddStaticTf() : Node("AddStaticTf") {

    // Declare and get parameters
    this->declare_parameter("SensorInTibia_RPY", "");
    this->declare_parameter("KneeInTibia_RPY", "");

    // Input is a list of 6 double, of the form [x, y, z, roll, pitch, yaw]
    sensor_param_ = this->get_parameter("SensorInTibia_RPY").as_string(); 
    knee_param_ = this->get_parameter("KneeInTibia_RPY").as_string(); 

    SensorInTibia_RPY_ = parse_double_array(sensor_param_);
    KneeInTibia_RPY_ = parse_double_array(knee_param_); 
        
    // Initialize Broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this); 

    // Publish transforms
    publish_tf(SensorInTibia_RPY_, "tibia_ref", "sensor_ref");
    publish_tf(KneeInTibia_RPY_, "tibia_ref", "knee_ref");

    RCLCPP_INFO(this->get_logger(), "Finished publishing static transforms to knee and sensor");

}

std::vector<double> AddStaticTf::parse_double_array(const std::string& str) {
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

void AddStaticTf::publish_tf(const std::vector<double>& tf_RPY, 
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
    rclcpp::spin(std::make_shared<AddStaticTf>());
    rclcpp::shutdown(); 
    
    return 0;
}