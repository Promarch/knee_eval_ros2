#include "calc_force_knee/visualize_force.hpp"

ForceVisualizer::ForceVisualizer() : Node("ForceVisualizer") {

    // Declare and get parameters
    this->declare_parameter("target_frame", "knee_ref");
    this->declare_parameter("source_frame", "map");
    target_frame_ = this->get_parameter("target_frame").as_string(); 
    source_frame_ = this->get_parameter("source_frame").as_string(); 
    
    // Initialize frame listeners
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

    // Initialize marker publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/force_marker", 10);

    // Initialize and start force subscriber
    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/wrench_knee", 10, 
        std::bind(&ForceVisualizer::ForceCallback, this, std::placeholders::_1)); 

    RCLCPP_INFO(this->get_logger(), "Node has been started"); 
}

void ForceVisualizer::ForceCallback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {

    // Create marker
    tf2::Vector3 force(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    visualization_msgs::msg::Marker marker_out = CreateMarker(force); 

    // Publish marker
    marker_pub_->publish(marker_out); 
}

visualization_msgs::msg::Marker ForceVisualizer::CreateMarker(const tf2::Vector3& force) {

    // Create starting point of the arrow
    std::vector<geometry_msgs::msg::Point> arrow_pts;
    geometry_msgs::msg::Point starting_point; 
    starting_point.x = 0;
    starting_point.y = 0;
    starting_point.z = 0;
    arrow_pts.push_back(starting_point); 

    // Create endpoint with scaling
    geometry_msgs::msg::Point end_pt; 
    int scaling_factor = 2; 
    end_pt.x = force[0]*scaling_factor; 
    end_pt.y = force[1]*scaling_factor; 
    end_pt.z = force[2]*scaling_factor; 
    arrow_pts.push_back(end_pt); 

    // Create marker
    visualization_msgs::msg::Marker force_marker;
    force_marker.header.frame_id = target_frame_; 
    force_marker.header.stamp = this->now(); 
    force_marker.type = visualization_msgs::msg::Marker::ARROW;
    force_marker.action = visualization_msgs::msg::Marker::ADD; 
    force_marker.points = arrow_pts;

    // Set arrow size
    force_marker.scale.x = 0.05;
    force_marker.scale.y = 0.05;
    force_marker.scale.z = 0.05;

    // Set marker color based on magnitude
    double magnitude = force.length(); 
    force_marker.color.r = std::min(1.0, magnitude / 30.0);
    force_marker.color.g = 0.0;
    force_marker.color.b = 1.0 - std::min(1.0, magnitude / 30.0);
    force_marker.color.a = 1.0;

    return force_marker; 
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<ForceVisualizer>()); 
    rclcpp::shutdown(); 
    return 0;
}