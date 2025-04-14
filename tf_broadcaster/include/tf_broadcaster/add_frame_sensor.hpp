#ifndef ADD_FRAME_SENSOR_HPP_
#define ADD_FRAME_SENSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

class AddFrameSensor : public rclcpp::Node {
public: 
    AddFrameSensor();

private: 
    void load_points_from_config(const std::string& filename, 
                                std::vector<Eigen::Vector3d>& marker_points,
                                std::vector<Eigen::Vector3d>& reference_points);

    void kabsch_algorithm(const std::vector<Eigen::Vector3d>& marker_points, 
                        const std::vector<Eigen::Vector3d>& reference_points, 
                        Eigen::Matrix3d& rotation, 
                        Eigen::Vector3d& translation);

    void publish_static_transform();

    // Member variables
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    std::vector<Eigen::Vector3d> marker_points_;
    std::vector<Eigen::Vector3d> reference_points_;

    std::string marker_frame_;
    std::string target_frame_;

    std::string marker_file_path_; 
};


#endif  // ADD_FRAME_SENSOR_HPP_