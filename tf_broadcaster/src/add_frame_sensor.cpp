#include <iostream>
#include "tf_broadcaster/add_frame_sensor.hpp"

AddFrameSensor::AddFrameSensor() : Node("AddFrameSensor") {
    // Declare and get parameters
    this->declare_parameter("marker_file_path", "");
    this->declare_parameter("marker_frame", "");
    this->declare_parameter("target_frame", "");

    marker_file_path_ = this->get_parameter("marker_file_path").as_string();
    marker_frame_ = this->get_parameter("marker_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    RCLCPP_INFO(this->get_logger(), "Marker_frame after declaring %s", marker_frame_.c_str());

    // Create static transform broadcaster
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // Load Points from config file
    load_points_from_config(marker_file_path_, marker_points_, reference_points_);

    // log the numbers of points loaded
    RCLCPP_INFO(this->get_logger(), "Loaded %zu marker points and %zu reference points", 
                marker_points_.size(), reference_points_.size());

    // Publish the static transform
    publish_static_transform(); 

    RCLCPP_INFO(this->get_logger(), "Static transform published");
}

void AddFrameSensor::load_points_from_config(const std::string& filename, 
                                            std::vector<Eigen::Vector3d>& marker_points, 
                                            std::vector<Eigen::Vector3d>& reference_points) {
    try {
        YAML::Node config = YAML::LoadFile(filename);

        // Load markers
        YAML::Node marker_coordinates = config["tibia_marker"];
        if (marker_coordinates.IsSequence()) {
            for (const auto& point : marker_coordinates) {
                double x = point["x"].as<double>();
                double y = point["y"].as<double>();
                double z = point["z"].as<double>();

                marker_points.push_back(Eigen::Vector3d(x,y,z));
            }
        }

        // Load tibia reference
        YAML::Node reference_coordinates = config["tibia_ref"];
        if (reference_coordinates.IsSequence()) {
            for (const auto& point : reference_coordinates) {
                double x = point["x"].as<double>();
                double y = point["y"].as<double>();
                double z = point["z"].as<double>();

                reference_points.push_back(Eigen::Vector3d(x,y,z));
            }
        }
    }
    catch (const YAML::Exception& e){
        RCLCPP_ERROR(this->get_logger(), "Error parsing YAML config file: %s", e.what()); 
    }
}

void AddFrameSensor::publish_static_transform() {
    // Compute the transformation using Kabsch algorithm
    Eigen::Matrix3d rotation; 
    Eigen::Vector3d translation; 
    
    if (marker_points_.size() >= 3 && reference_points_.size() >= 3 &&
        marker_points_.size() == reference_points_.size()) {
        
        // Compute transformation matrix
        kabsch_algorithm(marker_points_, reference_points_, rotation, translation); 

        // Create the transform
        geometry_msgs::msg::TransformStamped transform_stamped; 
        transform_stamped.header.stamp = this->now(); 
        transform_stamped.header.frame_id = marker_frame_; 
        transform_stamped.child_frame_id = target_frame_;
        
        // Set translation
        transform_stamped.transform.translation.x = translation.x();
        transform_stamped.transform.translation.y = translation.y();
        transform_stamped.transform.translation.z = translation.z();

        // Convert rotation matrix to quaternion
        Eigen::Quaterniond q(rotation); 
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // Publish static transform
        static_broadcaster_->sendTransform(transform_stamped); 

        RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s", marker_frame_.c_str(), target_frame_.c_str()); 
    }
    else {
        RCLCPP_ERROR(this->get_logger(), 
                        "Not enough points or point count missmatch: Marker points: %zu, Reference points %zu", 
                        marker_points_.size(), reference_points_.size());
    }
}

void AddFrameSensor::kabsch_algorithm(
    const std::vector<Eigen::Vector3d>& marker_points, 
    const std::vector<Eigen::Vector3d>& reference_points,
    Eigen::Matrix3d& rotation, 
    Eigen::Vector3d& translation) 
{

    // Check if we have enough points and match number of points
    size_t n = marker_points.size(); 
    if ( n<3 || n!=reference_points.size()) {
        RCLCPP_ERROR(this->get_logger(), "Need at least 3 matching points for Kabsch algorithm"); 
        rotation = Eigen::Matrix3d::Identity(); 
        translation = Eigen::Vector3d::Zero(); 
        return; 
    }

    // Create Matrices to holf point coordinates
    Eigen::MatrixXd P(3,n); // Marker Points
    Eigen::MatrixXd Q(3,n); // Reference Points

    // Fill Matrices with points coordinates
    for (size_t i=0; i<n; i++) {
        P.col(i) = marker_points[i]; 
        Q.col(i) = reference_points[i];
    }

    // Calculate centroids
    Eigen::Vector3d centroid_P = P.rowwise().mean();
    Eigen::Vector3d centroid_Q = Q.rowwise().mean(); 

    // Center the points
    Eigen::MatrixXd P_centered = P.colwise() - centroid_P;
    Eigen::MatrixXd Q_centered = Q.colwise() - centroid_Q;
    
    // Compute covariance matrix H
    Eigen::Matrix3d H = P_centered * Q_centered.transpose(); 

    // Compute SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Determine the opimal rotation
    rotation = svd.matrixV() * svd.matrixU().transpose(); 

    // Ensure we have proper rotation matrix (det = +1)
    if (rotation.determinant() <0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        rotation = V * svd.matrixU().transpose();
    }

    // Compute the translation
    translation = centroid_Q - rotation * centroid_P;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<AddFrameSensor>());
    rclcpp::shutdown();
    return 0;
}