/**
 * @file v4l2_stereo_node.cpp
 * @brief V4L2-based stereo camera publisher (fallback)
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
#include <filesystem>

namespace stereo_camera {

class V4L2StereoNode : public rclcpp::Node {
public:
    V4L2StereoNode() : Node("v4l2_stereo_node") {
        // Declare parameters
        this->declare_parameter("device_left", "/dev/video0");
        this->declare_parameter("device_right", "/dev/video1");
        this->declare_parameter("width", 1280);
        this->declare_parameter("height", 720);
        this->declare_parameter("fps", 30);
        this->declare_parameter("camera_info_url_left", "");
        this->declare_parameter("camera_info_url_right", "");
        this->declare_parameter("auto_detect", false);
        
        // Get parameters
        device_left_ = this->get_parameter("device_left").as_string();
        device_right_ = this->get_parameter("device_right").as_string();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        bool auto_detect = this->get_parameter("auto_detect").as_bool();
        
        if (auto_detect) {
            autoDetectDevices();
        }
        
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
        left_thread_ = std::thread(&V4L2StereoNode::captureLoop, this, device_left_, true);
        right_thread_ = std::thread(&V4L2StereoNode::captureLoop, this, device_right_, false);
        
        RCLCPP_INFO(this->get_logger(), "V4L2 stereo node started (%s, %s @ %dx%d/%d fps)",
                   device_left_.c_str(), device_right_.c_str(), width_, height_, fps_);
    }
    
    ~V4L2StereoNode() {
        running_ = false;
        if (left_thread_.joinable()) left_thread_.join();
        if (right_thread_.joinable()) right_thread_.join();
    }

private:
    void autoDetectDevices() {
        // Scan /dev for video devices
        std::vector<std::string> devices;
        for (int i = 0; i < 10; ++i) {
            std::string dev = "/dev/video" + std::to_string(i);
            if (std::filesystem::exists(dev)) {
                devices.push_back(dev);
            }
        }
        
        if (devices.size() >= 2) {
            device_left_ = devices[0];
            device_right_ = devices[1];
            RCLCPP_INFO(this->get_logger(), "Auto-detected devices: %s, %s",
                       device_left_.c_str(), device_right_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not auto-detect stereo pair, using defaults");
        }
    }
    
    void captureLoop(const std::string& device, bool is_left) {
        RCLCPP_INFO(this->get_logger(), "Opening %s camera: %s",
                   is_left ? "left" : "right", device.c_str());
        
        cv::VideoCapture cap(device, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s camera: %s",
                        is_left ? "left" : "right", device.c_str());
            return;
        }
        
        // Set camera properties
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap.set(cv::CAP_PROP_FPS, fps_);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        
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
            if (frame.channels() == 3) {
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = frame;
            }
            
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
    
    std::string device_left_, device_right_;
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
    auto node = std::make_shared<stereo_camera::V4L2StereoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
