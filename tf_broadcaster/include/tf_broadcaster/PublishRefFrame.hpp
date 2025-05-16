#ifndef ADD_FRAME_SENSOR_HPP_
#define ADD_FRAME_SENSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

class PublishRefFrame : public rclcpp::Node {
public: 
    PublishRefFrame();

private: 

    // Process all transformations in the YAML file
    void process_all_transformations(const std::string& filename);

    // Load points for a specific marker-reference pair
    void load_points_from_config(const YAML::Node& config, 
                                const std::string& marker_frame, 
                                const std::string& target_frame, 
                                std::vector<Eigen::Vector3d>& marker_points,
                                std::vector<Eigen::Vector3d>& reference_points);

    void kabsch_algorithm(const std::vector<Eigen::Vector3d>& marker_points, 
                        const std::vector<Eigen::Vector3d>& reference_points, 
                        Eigen::Matrix3d& rotation, 
                        Eigen::Vector3d& translation);

    void publish_static_transform(const std::string& marker_frame, 
                                const std::string& target_frame, 
                                const std::vector<Eigen::Vector3d>& marker_points, 
                                const std::vector<Eigen::Vector3d>& reference_points);

    // Member variables
    std::string marker_file_path_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

};


#endif  // ADD_FRAME_SENSOR_HPP_