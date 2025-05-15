#include "csv_writer/csv_writer.hpp"

CsvWriter::CsvWriter(const rclcpp::NodeOptions& options) : 
    rclcpp_lifecycle::LifecycleNode("CsvWriter", options), is_recording_(false), transforms_available_(false) {

    RCLCPP_INFO(this->get_logger(), "CSV Writer Lifecycle node has been instantiated");
}

CallbackReturn CsvWriter::on_configure(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Configuring [%s] from [%s] state...", this->get_name(), state.label().c_str());

    // Declare and get parameters
    this->declare_parameter("path_csv_file", "data.csv");
    this->declare_parameter("source_frame", "optitrack");
    this->declare_parameter("tibia_frame", "tibia_ref");
    this->declare_parameter("femur_frame", "femur_ref");
    this->declare_parameter("knee_frame", "knee_ref");

    path_csv_file_ = this->get_parameter("path_csv_file").as_string(); 
    source_frame_ = this->get_parameter("source_frame").as_string(); 
    tibia_frame_ = this->get_parameter("tibia_frame").as_string(); 
    femur_frame_ = this->get_parameter("femur_frame").as_string(); 
    knee_frame_ = this->get_parameter("knee_frame").as_string(); 

    // Initialize frame listeners
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

    // Create services
    start_recording_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_csv_recording", 
        std::bind(&CsvWriter::HandleStartRecording, this, std::placeholders::_1, std::placeholders::_2));
    stop_recording_service_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_csv_recording", 
        std::bind(&CsvWriter::HandleStopRecording, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Configuration complete. Parameters loaded");

    return CallbackReturn::SUCCESS; 
}

// Transition from inactiva to active
CallbackReturn CsvWriter::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Activating [%s] from [%s] state...", this->get_name(), state.label().c_str());

    // Wait for the transformation to be available
    rclcpp::Rate rate(1); 
    bool transform_available = false; 
    while (rclcpp::ok() && !transform_available) {
        try {
            if (tf_buffer_->canTransform(tibia_frame_, source_frame_, tf2::TimePointZero)) {
                transform_available = true; 
                RCLCPP_INFO(this->get_logger(), "Transform between %s and %s is now available", 
                    tibia_frame_.c_str(), source_frame_.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Waiting for transform between %s and %s...", 
                    tibia_frame_.c_str(), source_frame_.c_str());
                rate.sleep();
            }
        }
        catch (const tf2::TransformException& e) {
            RCLCPP_INFO(this->get_logger(), "Exception while waiting for transform: %s", e.what());
            rate.sleep();
        }
    }

    // Timer instead of subscriber
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 100Hz = 10ms period
        std::bind(&CsvWriter::TimerCallback, this));
/*    // Initialize the wrench subscriber
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/wrench_knee", 10, 
    std::bind(&CsvWriter::ForceCallback, this, std::placeholders::_1)); 
*/

    RCLCPP_INFO(this->get_logger(), "CSV Writer now active and waiting for start_csv_recording service call");
    RCLCPP_INFO(this->get_logger(), "To start recording, use: ros2 service call /start_csv_recording std_srvs/srv/Trigger {}");

    return CallbackReturn::SUCCESS;
}

// Transition from activate to inactive
CallbackReturn CsvWriter::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Deactivating [%s] from [%s] state...", this->get_name(), state.label().c_str());

    // Stop recording if active
    if (is_recording_ && csv_file_.is_open()) {
        csv_file_.close();
        is_recording_ = false;
        RCLCPP_INFO(this->get_logger(), "Recording stopped during deactivation");
    }
    
    // Cancel timer
    timer_.reset();
    
    // Reset subscriber if it was created
    // wrench_sub_.reset();
    
    RCLCPP_INFO(this->get_logger(), "Node deactivated");

    return CallbackReturn::SUCCESS;
}

// Transition from inactive to unconfigured
CallbackReturn CsvWriter::on_cleanup(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Cleaning up [%s] from [%s] state...", this->get_name(), state.label().c_str());

    // Clean up transform listener and buffer
    tf_listener_.reset(); 
    tf_buffer_.reset(); 

    transforms_available_ = false; 

    RCLCPP_INFO(this->get_logger(), "Cleanup complete"); 

    return CallbackReturn::SUCCESS; 
}

// Shut down the node
CallbackReturn CsvWriter::on_shutdown(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(this->get_logger(), "Shutting down [%s] from [%s] state...", this->get_name(), state.label().c_str());

    // Ensure file is closed
    if (csv_file_.is_open()) {
        csv_file_.close();
        is_recording_ = false;
        RCLCPP_INFO(this->get_logger(), "CSV file closed during shutdown");
    }

    rclcpp::shutdown();

    return CallbackReturn::SUCCESS; 
}

// Called when an error occurs
CallbackReturn CsvWriter::on_error(const rclcpp_lifecycle::State& state) {
    RCLCPP_ERROR(this->get_logger(), "Error occured in [%s] state", state.label().c_str());
    // Ensure file is closed
    if (csv_file_.is_open()) {
        csv_file_.close();
        is_recording_ = false;
        RCLCPP_INFO(this->get_logger(), "CSV file closed after error");
    }

    return CallbackReturn::SUCCESS; 
}

void CsvWriter::HandleStartRecording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, 
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    // Check if the node is in an active state
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        response->success = false; 
        response->message = "Node is not in active state. Cannot start recording"; 
        return; 
    }
    // If the file is already recording don't do anything
    if (is_recording_) {
        response->success = false; 
        response->message = "Already recording to csv file: " + path_csv_file_; 
        return; 
    }

    // Open CSV
    csv_file_.open(path_csv_file_, std::ios::out); 
    if (csv_file_.is_open()) {
        is_recording_ = true; 
        RCLCPP_INFO(this->get_logger(), "Started recording to CSV file: %s", path_csv_file_.c_str()); 
        InitializeCsv(); 
        response->success = true; 
        response->message = "Succesfully started recording to: " + path_csv_file_; 
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "CSV file could not be opended: %s", path_csv_file_.c_str()); 
        response->success = false; 
        response->message = "Failed to open CSV file: " + path_csv_file_; 
    }
}

void CsvWriter::HandleStopRecording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/, 
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    // Dont do anything if not recording
    if (!is_recording_) {
        response->success = false; 
        response->message = "Not currently recording"; 
        return; 
    }

    // Close CSV
    if (csv_file_.is_open()) {
        csv_file_.close(); 
        is_recording_ = false; 
        RCLCPP_INFO(this->get_logger(), "Stopped recording to csv file: %s", path_csv_file_.c_str()); 
        response->success = true; 
        response->message = "Succesfully stopped recording to " + path_csv_file_; 
    }
    else {
        response->success = false; 
        response->message = "CSV file was not open"; 
    }
}

void CsvWriter::TimerCallback() {

    // Skip if node is not active
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return; 
    }
    // Skip if not recording
    if (!is_recording_ || !csv_file_.is_open()) {
        return; 
    }

    try {
        // Write header and zero forces (replacing the received forces)
        rclcpp::Time current_time = this->now();
        std::string timestamp = std::to_string(current_time.seconds()); 
        csv_file_ << timestamp << ","; 
        csv_file_ << "0.0,0.0,0.0,";
        csv_file_ << "0.0,0.0,0.0,";

        // Write tf from source to tibia
        geometry_msgs::msg::TransformStamped tf_tibia; 
        tf_tibia = tf_buffer_->lookupTransform(source_frame_, tibia_frame_, tf2::TimePointZero); 
        RCLCPP_DEBUG(this->get_logger(), "Succesfully retrieved transform from %s to %s", 
                                            tibia_frame_.c_str(), source_frame_.c_str()); 
        WriteTf2Csv(tf_tibia); 

        // Write tf from source to femur
        geometry_msgs::msg::TransformStamped tf_femur; 
        tf_femur = tf_buffer_->lookupTransform(source_frame_, femur_frame_, tf2::TimePointZero); 
        RCLCPP_DEBUG(this->get_logger(), "Succesfully retrieved transform from %s to %s", 
                                            femur_frame_.c_str(), source_frame_.c_str()); 
        RCLCPP_DEBUG(this->get_logger(), "Femur: x=%f, y=%f, z=%f", tf_femur.transform.translation.x, tf_femur.transform.translation.y, tf_femur.transform.translation.z);
        RCLCPP_DEBUG(this->get_logger(), "Source frame: %s", source_frame_.c_str());
        WriteTf2Csv(tf_femur); 

        // Write tf from source to femur
        geometry_msgs::msg::TransformStamped tf_knee; 
        tf_knee = tf_buffer_->lookupTransform(knee_frame_, source_frame_, tf2::TimePointZero); 
        RCLCPP_DEBUG(this->get_logger(), "Succesfully retrieved transform from %s to %s", 
                                            knee_frame_.c_str(), source_frame_.c_str()); 
        WriteTf2Csv(tf_knee); 

        csv_file_ << std::endl; 

        // Flush the file to ensure data is writte immediately
        csv_file_.flush(); 
    }
    catch (const tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s; %s", source_frame_.c_str(), tibia_frame_.c_str(), e.what());
        csv_file_ << std::endl << "transform error"; 
        return; 
    }
}


void CsvWriter::ForceCallback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg) {

    // Skip if node is not active
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return;
    }
    // Skip if not recording
    if (!is_recording_ || !csv_file_.is_open()) {
        return; 
    }

    try {
        // Write header and received forces
        rclcpp::Time current_time = msg->header.stamp; 
        std::string timestamp = std::to_string(current_time.seconds()); 
        csv_file_ << timestamp << ","; 
        csv_file_ << msg->wrench.force.x << "," << msg->wrench.force.y << "," << msg->wrench.force.z << ",";
        csv_file_ << msg->wrench.torque.x << "," << msg->wrench.torque.y << "," << msg->wrench.torque.z << ",";

        // Write tf from source to tibia
        geometry_msgs::msg::TransformStamped tf_tibia; 
        tf_tibia = tf_buffer_->lookupTransform(source_frame_, tibia_frame_, tf2::TimePointZero); 
        RCLCPP_DEBUG(this->get_logger(), "Succesfully retrieved transform from %s to %s", 
                                            tibia_frame_.c_str(), source_frame_.c_str()); 
        WriteTf2Csv(tf_tibia); 

        // Write tf from source to femur
        geometry_msgs::msg::TransformStamped tf_femur; 
        tf_femur = tf_buffer_->lookupTransform(source_frame_, femur_frame_, tf2::TimePointZero); 
        RCLCPP_DEBUG(this->get_logger(), "Succesfully retrieved transform from %s to %s", 
                                            femur_frame_.c_str(), source_frame_.c_str()); 
        WriteTf2Csv(tf_femur); 

        // Write tf from source to femur
        geometry_msgs::msg::TransformStamped tf_knee; 
        tf_knee = tf_buffer_->lookupTransform(source_frame_, knee_frame_, tf2::TimePointZero); 
        RCLCPP_DEBUG(this->get_logger(), "Succesfully retrieved transform from %s to %s", 
                                            knee_frame_.c_str(), source_frame_.c_str()); 
        WriteTf2Csv(tf_knee); 

        csv_file_ << std::endl; 

        // Flush the file to ensure data is writte immediately
        csv_file_.flush(); 
    }
    catch (const tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s; %s", source_frame_.c_str(), tibia_frame_.c_str(), e.what());
        csv_file_ << std::endl << "transform error"; 
        return; 
    }
}

void CsvWriter::InitializeCsv() {

    // Write CSV header
    csv_file_ << "timestamp,"; 
    csv_file_ << "force_x,force_y,force_z,"; 
    csv_file_ << "torque_x, torque_y, torque_z,";
    csv_file_ << "tibia_x, tibia_y, tibia_z,"; 
    csv_file_ << "tibia_qx, tibia_qy, tibia_qz, tibia_qw,";
    csv_file_ << "femur_x, femur_y, femur_z,"; 
    csv_file_ << "femur_qx, femur_qy, femur_qz, femur_qw,";
    csv_file_ << "knee_x, knee_y, knee_z,"; 
    csv_file_ << "knee_qx, knee_qy, knee_qz, knee_qw" << std::endl;

}

void CsvWriter::WriteTf2Csv(const geometry_msgs::msg::TransformStamped& tf) {

    // Write content of tf to csv
    csv_file_ << tf.transform.translation.x << ","
              << tf.transform.translation.y << ","
              << tf.transform.translation.z << ",";
    csv_file_ << tf.transform.rotation.x << ","
              << tf.transform.rotation.y << ","
              << tf.transform.rotation.z << ","
              << tf.transform.rotation.w << ",";

}

CsvWriter::~CsvWriter() {
    // Close csv file when node is destroyed
    if (csv_file_.is_open()) {
        csv_file_.close(); 
        RCLCPP_INFO(this->get_logger(), "CSV file closed in destructor"); 
    }
}

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor; 
    std::shared_ptr<CsvWriter> node = std::make_shared<CsvWriter>(); 

    executor.add_node(node->get_node_base_interface()); 
    executor.spin(); 

    rclcpp::shutdown(); 
    
    return 0;
}