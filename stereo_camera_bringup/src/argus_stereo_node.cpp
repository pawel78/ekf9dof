/**
 * @file argus_stereo_node.cpp
 * @brief Argus-based stereo camera publisher for Jetson
 * 
 * Uses nvarguscamerasrc GStreamer element to capture from synchronized stereo cameras
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <thread>
#include <atomic>

namespace stereo_camera {

class ArgusStereoNode : public rclcpp::Node {
public:
    ArgusStereoNode() : Node("argus_stereo_node") {
        // Declare parameters
        this->declare_parameter("sensor_id_left", 0);
        this->declare_parameter("sensor_id_right", 1);
        this->declare_parameter("width", 1280);
        this->declare_parameter("height", 720);
        this->declare_parameter("fps", 30);
        this->declare_parameter("camera_info_url_left", "");
        this->declare_parameter("camera_info_url_right", "");
        
        // Get parameters
        sensor_id_left_ = this->get_parameter("sensor_id_left").as_int();
        sensor_id_right_ = this->get_parameter("sensor_id_right").as_int();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        
        std::string info_url_left = this->get_parameter("camera_info_url_left").as_string();
        std::string info_url_right = this->get_parameter("camera_info_url_right").as_string();
        
        // Create publishers
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/stereo/left/image_raw", 10);
        right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/stereo/right/image_raw", 10);
        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/stereo/left/camera_info", 10);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/stereo/right/camera_info", 10);
        
        // Initialize camera info managers
        left_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this, "left_camera", info_url_left);
        right_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this, "right_camera", info_url_right);
        
        // Start capture threads
        running_ = true;
        left_thread_ = std::thread(&ArgusStereoNode::captureLoop, this, sensor_id_left_, true);
        right_thread_ = std::thread(&ArgusStereoNode::captureLoop, this, sensor_id_right_, false);
        
        RCLCPP_INFO(this->get_logger(), "Argus stereo node started (sensors %d, %d @ %dx%d/%d fps)",
                   sensor_id_left_, sensor_id_right_, width_, height_, fps_);
    }
    
    ~ArgusStereoNode() {
        running_ = false;
        if (left_thread_.joinable()) left_thread_.join();
        if (right_thread_.joinable()) right_thread_.join();
    }

private:
    void captureLoop(int sensor_id, bool is_left) {
        // Build GStreamer pipeline for nvarguscamerasrc
        std::stringstream pipeline_ss;
        pipeline_ss << "nvarguscamerasrc sensor-id=" << sensor_id
                   << " ! video/x-raw(memory:NVMM), width=" << width_
                   << ", height=" << height_ << ", framerate=" << fps_ << "/1"
                   << " ! nvvidconv ! video/x-raw, format=BGRx"
                   << " ! videoconvert ! video/x-raw, format=BGR"
                   << " ! appsink";
        
        std::string pipeline = pipeline_ss.str();
        RCLCPP_INFO(this->get_logger(), "Opening %s camera with pipeline: %s",
                   is_left ? "left" : "right", pipeline.c_str());
        
        cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s camera (sensor %d)",
                        is_left ? "left" : "right", sensor_id);
            return;
        }
        
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            if (!cap.read(frame) || frame.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Failed to capture frame from %s camera",
                                    is_left ? "left" : "right");
                continue;
            }
            
            // Convert to grayscale for VO
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            
            // Create ROS message
            auto now = this->now();
            auto img_msg = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                sensor_msgs::image_encodings::MONO8,
                gray).toImageMsg();
            img_msg->header.stamp = now;
            img_msg->header.frame_id = is_left ? "left_camera" : "right_camera";
            
            // Get camera info
            auto info_msg = is_left ? 
                left_info_manager_->getCameraInfo() :
                right_info_manager_->getCameraInfo();
            info_msg.header = img_msg->header;
            
            // Publish
            if (is_left) {
                left_image_pub_->publish(*img_msg);
                left_info_pub_->publish(info_msg);
            } else {
                right_image_pub_->publish(*img_msg);
                right_info_pub_->publish(info_msg);
            }
        }
    }
    
    int sensor_id_left_, sensor_id_right_;
    int width_, height_, fps_;
    std::atomic<bool> running_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
    
    std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;
    
    std::thread left_thread_;
    std::thread right_thread_;
};

} // namespace stereo_camera

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereo_camera::ArgusStereoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
