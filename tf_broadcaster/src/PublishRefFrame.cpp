#include <iostream>
#include "tf_broadcaster/PublishRefFrame.hpp"

PublishRefFrame::PublishRefFrame() : Node("PublishRefFrame") {
    // Declare and get parameters
    this->declare_parameter("marker_file_path", "");
    marker_file_path_ = this->get_parameter("marker_file_path").as_string();

    // Create static transform broadcaster
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // Load Points from config file
    process_all_transformations(marker_file_path_);

    RCLCPP_INFO(this->get_logger(), "Finished publishing transforms from optitrack to tracker frames");
}

void PublishRefFrame::process_all_transformations(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename); 
        // Find all keys that end with _body to identify markers
        std::vector<std::string> marker_frames;
        for (const auto& entry : config) {
            std::string key = entry.first.as<std::string>(); 
            if (key.length() > 5 && key.substr(key.length() - 5) == "_body") {
                marker_frames.push_back(key); 
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Found %zu marker frames to process", marker_frames.size()); 

        // Process each marker-reference pair
        for (const auto& marker_frame : marker_frames) {
            // Derive reference frame name by replacing _body with _ref
            std::string target_frame = marker_frame.substr(0, marker_frame.length() - 5) + "_ref";
            
            // Check if both marker and reference exist in the config
            if (config[marker_frame] && config[target_frame]) {
                std::vector<Eigen::Vector3d> marker_points; 
                std::vector<Eigen::Vector3d> reference_points; 

                // Load the points
                load_points_from_config(config, marker_frame, target_frame, marker_points, reference_points);

                // Publish transformation if points were loaded succesfully
                if (!marker_points.empty() && !reference_points.empty()) {
                    publish_static_transform(marker_frame, target_frame, marker_points, reference_points);
                }
                else {
                    RCLCPP_WARN(this->get_logger(), "Marker points or reference points are empty. Marker pts: %zu, Ref pts: %zu", 
                        marker_points.size(), reference_points.size());
                }
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Missing corresponding reference frame for %s", marker_frame.c_str()); 
            }
        }
    }
    catch (YAML::Exception& e){
        RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what()); 
    }
}

void PublishRefFrame::load_points_from_config(const YAML::Node& config, 
                                            const std::string& marker_frame, 
                                            const std::string& target_frame,
                                            std::vector<Eigen::Vector3d>& marker_points, 
                                            std::vector<Eigen::Vector3d>& reference_points) {

    // clear any existing points
    marker_points.clear(); 
    reference_points.clear(); 

    // Load markers
    YAML::Node marker_coordinates = config[marker_frame];
    if (marker_coordinates.IsSequence()) {
        for (const auto& point : marker_coordinates) {
            double x = point["x"].as<double>();
            double y = point["y"].as<double>();
            double z = point["z"].as<double>();

            marker_points.push_back(Eigen::Vector3d(x,y,z));
        }
    }

    // Load reference
    YAML::Node reference_coordinates = config[target_frame];
    if (marker_coordinates.IsSequence()) {
        for (const auto& point : reference_coordinates) {
            double x = point["x"].as<double>();
            double y = point["y"].as<double>();
            double z = point["z"].as<double>();

            reference_points.push_back(Eigen::Vector3d(x,y,z));
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Loaded %zu marker points and %zu reference points for %s to %s", 
        marker_points.size(), reference_points.size(), marker_frame.c_str(), target_frame.c_str()); 
}

void PublishRefFrame::publish_static_transform(const std::string& marker_frame, 
                                            const std::string& target_frame, 
                                            const std::vector<Eigen::Vector3d>& marker_points, 
                                            const std::vector<Eigen::Vector3d>& reference_points) {
    // Compute the transformation using Kabsch algorithm
    Eigen::Matrix3d rotation; 
    Eigen::Vector3d translation; 
    
    if (marker_points.size() >= 3 && reference_points.size() >= 3 &&
        marker_points.size() == reference_points.size()) {
        
        // Compute transformation matrix
        kabsch_algorithm(marker_points, reference_points, rotation, translation); 

        // Create the transform
        geometry_msgs::msg::TransformStamped transform_stamped; 
        transform_stamped.header.stamp = this->now(); 
        transform_stamped.header.frame_id = marker_frame; 
        transform_stamped.child_frame_id = target_frame;
        
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

        RCLCPP_DEBUG(this->get_logger(), "Published static transform from %s to %s", marker_frame.c_str(), target_frame.c_str()); 
    }
    else {
        RCLCPP_ERROR(this->get_logger(), 
                        "Not enough points or point count mismatch for %s to %s: Marker points: %zu, Reference points %zu", 
                        marker_frame.c_str(), target_frame.c_str(), marker_points.size(), reference_points.size());
    }
}

void printEigenMatrix(const Eigen::MatrixXd &matrix, const rclcpp::Logger &logger)
{
  for (int i = 0; i < matrix.rows(); ++i)
  {
    std::ostringstream row_stream;
    for (int j = 0; j < matrix.cols(); ++j)
    {
      row_stream << matrix(i, j);
      if (j < matrix.cols() - 1)
        row_stream << ", ";
    }
    RCLCPP_DEBUG(logger, "[%s]", row_stream.str().c_str());
  }
}

void PublishRefFrame::kabsch_algorithm(
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

    // Create Matrices to hold point coordinates: Algorithm return transformation from Q to P
    Eigen::MatrixXd Q(n,3); // Marker Points
    Eigen::MatrixXd P(n,3); // Reference Points

    // Fill Matrices with points coordinates
    for (size_t i=0; i<n; i++) {
        Q.row(i) = reference_points[i];
        P.row(i) = marker_points[i]; 
    }

    // Calculate centroids
    Eigen::Vector3d centroid_Q = Q.colwise().mean(); 
    Eigen::Vector3d centroid_P = P.colwise().mean();
    
    // Center the points
    Eigen::MatrixXd Q_centered = Q.rowwise() - centroid_Q.transpose();
    Eigen::MatrixXd P_centered = P.rowwise() - centroid_P.transpose();
    // printEigenMatrix(P_centered, this->get_logger());

    // Compute covariance matrix H
    Eigen::Matrix3d H = P_centered.transpose() * Q_centered; 

    // Compute SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Determine the opimal rotation
    rotation = svd.matrixU() * svd.matrixV().transpose(); 
    RCLCPP_DEBUG(this->get_logger(), "Rotation matrix");
    printEigenMatrix(rotation, this->get_logger());

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
    rclcpp::spin(std::make_shared<PublishRefFrame>());
    rclcpp::shutdown();
    return 0;
}