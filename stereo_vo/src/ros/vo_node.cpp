/**
 * @file vo_node.cpp
 * @brief Main ROS 2 node for stereo visual odometry
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>

#include "stereo_vo/core/feature_tracker.hpp"
#include "stereo_vo/core/stereo_matcher.hpp"
#include "stereo_vo/core/pose_estimator.hpp"
#include "stereo_vo/core/sliding_window.hpp"
#include "stereo_vo/core/imu_preintegrator.hpp"

namespace stereo_vo {
namespace ros {

class VONode : public rclcpp::Node {
public:
    VONode() : Node("vo_node"), frame_id_(0), initialized_(false) {
        // Declare parameters
        this->declare_parameter("use_imu", false);
        this->declare_parameter("max_features", 1500);
        this->declare_parameter("lk_levels", 3);
        this->declare_parameter("ransac_reproj_thresh_px", 2.0);
        this->declare_parameter("ba_window_size", 7);
        this->declare_parameter("publish_debug_images", true);
        
        // Get parameters
        use_imu_ = this->get_parameter("use_imu").as_bool();
        bool publish_debug = this->get_parameter("publish_debug_images").as_bool();
        
        // Initialize core components
        core::FeatureTracker::Config ft_config;
        ft_config.max_features = this->get_parameter("max_features").as_int();
        ft_config.lk_levels = this->get_parameter("lk_levels").as_int();
        feature_tracker_ = std::make_unique<core::FeatureTracker>(ft_config);
        
        core::PoseEstimator::Config pe_config;
        pe_config.ransac_reproj_thresh_px = this->get_parameter("ransac_reproj_thresh_px").as_double();
        pose_estimator_ = std::make_unique<core::PoseEstimator>(pe_config);
        
        stereo_matcher_ = std::make_unique<core::StereoMatcher>();
        
        core::SlidingWindow::Config sw_config;
        sw_config.window_size = this->get_parameter("ba_window_size").as_int();
        sw_config.use_imu = use_imu_;
        sliding_window_ = std::make_unique<core::SlidingWindow>(sw_config);
        
        if (use_imu_) {
            imu_preintegrator_ = std::make_unique<core::IMUPreintegrator>();
        }
        
        // Initialize pose
        current_R_ = Eigen::Matrix3d::Identity();
        current_t_ = Eigen::Vector3d::Zero();
        
        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/vo/odom", 10);
        quality_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vo/quality", 10);
        
        if (publish_debug) {
            tracks_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vo/tracks", 10);
        }
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Create subscribers for stereo images
        left_image_sub_.subscribe(this, "/stereo/left/image_raw");
        right_image_sub_.subscribe(this, "/stereo/right/image_raw");
        left_info_sub_.subscribe(this, "/stereo/left/camera_info");
        right_info_sub_.subscribe(this, "/stereo/right/camera_info");
        
        // Synchronize stereo pairs
        sync_ = std::make_shared<message_filters::TimeSynchronizer<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>>(
            left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_, 10);
        
        sync_->registerCallback(std::bind(&VONode::stereoCallback, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4));
        
        // IMU subscriber
        if (use_imu_) {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", 100,
                std::bind(&VONode::imuCallback, this, std::placeholders::_1));
        }
        
        RCLCPP_INFO(this->get_logger(), "VO Node initialized (use_imu=%s)", use_imu_ ? "true" : "false");
    }

private:
    void stereoCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info) {
        
        auto start_time = std::chrono::steady_clock::now();
        
        // Convert to OpenCV
        cv_bridge::CvImagePtr left_cv, right_cv;
        try {
            left_cv = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::MONO8);
            right_cv = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // Initialize camera parameters on first frame
        if (!cam_params_set_) {
            setCameraParams(left_info, right_info);
            cam_params_set_ = true;
        }
        
        // Track features in left image
        std::vector<core::FeatureTracker::Feature> features;
        feature_tracker_->processFrame(left_cv->image, features);
        
        if (features.size() < 50) {
            RCLCPP_WARN(this->get_logger(), "Too few features: %zu", features.size());
            return;
        }
        
        // Match stereo correspondences
        std::vector<cv::Point2f> pts_left, pts_right;
        for (const auto& feat : features) {
            pts_left.push_back(feat.pt);
        }
        
        std::vector<bool> stereo_mask;
        stereo_matcher_->matchFeatures(pts_left, right_cv->image, pts_right, stereo_mask);
        
        // Triangulate 3D points
        std::vector<cv::Point2f> valid_left, valid_right;
        for (size_t i = 0; i < stereo_mask.size(); ++i) {
            if (stereo_mask[i]) {
                valid_left.push_back(pts_left[i]);
                valid_right.push_back(pts_right[i]);
            }
        }
        
        std::vector<Eigen::Vector3d> points_3d;
        if (!valid_left.empty()) {
            stereo_matcher_->triangulate(valid_left, valid_right, points_3d);
        }
        
        // Estimate pose if not first frame
        if (initialized_ && !points_3d.empty() && !prev_points_3d_.empty()) {
            core::PoseEstimator::PoseResult result;
            pose_estimator_->estimatePose(prev_points_3d_, valid_left, K_, result);
            
            if (result.success) {
                // Update pose
                current_t_ = current_R_ * result.t + current_t_;
                current_R_ = current_R_ * result.R;
                
                // Add to sliding window
                core::SlidingWindow::Frame frame;
                frame.frame_id = frame_id_++;
                frame.R = current_R_;
                frame.t = current_t_;
                frame.points_3d = points_3d;
                frame.observations = valid_left;
                sliding_window_->addFrame(frame);
                
                // Optimize periodically
                if (frame_id_ % 5 == 0) {
                    sliding_window_->optimize();
                    sliding_window_->getPose(frame_id_ - 1, current_R_, current_t_);
                }
                
                // Publish odometry
                publishOdometry(left_msg->header.stamp, result.num_inliers);
                
                // Publish quality metric
                auto quality_msg = std_msgs::msg::Float32();
                quality_msg.data = static_cast<float>(result.num_inliers) / features.size();
                quality_pub_->publish(quality_msg);
            }
        } else {
            initialized_ = true;
        }
        
        // Store for next iteration
        prev_points_3d_ = points_3d;
        
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_DEBUG(this->get_logger(), "VO processing time: %ld ms, features: %zu",
                    duration.count(), features.size());
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        if (!imu_preintegrator_) return;
        
        core::IMUPreintegrator::ImuMeasurement meas;
        meas.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        meas.angular_velocity = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        meas.linear_acceleration = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
        
        imu_preintegrator_->addMeasurement(meas);
    }
    
    void setCameraParams(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
                        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info) {
        K_ = (cv::Mat_<double>(3, 3) <<
            left_info->k[0], left_info->k[1], left_info->k[2],
            left_info->k[3], left_info->k[4], left_info->k[5],
            left_info->k[6], left_info->k[7], left_info->k[8]);
        
        core::StereoMatcher::CameraParams params;
        params.K_left = K_.clone();
        params.K_right = K_.clone();
        params.baseline = 0.12; // 12 cm, should come from calibration
        stereo_matcher_->setCameraParams(params);
    }
    
    void publishOdometry(const rclcpp::Time& stamp, int num_inliers) {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Set position
        odom_msg.pose.pose.position.x = current_t_.x();
        odom_msg.pose.pose.position.y = current_t_.y();
        odom_msg.pose.pose.position.z = current_t_.z();
        
        // Convert rotation to quaternion
        Eigen::Quaterniond q(current_R_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // Set covariance based on inliers
        double cov = 0.01 * (100.0 / std::max(num_inliers, 1));
        for (int i = 0; i < 6; ++i) {
            odom_msg.pose.covariance[i * 6 + i] = cov;
        }
        
        odom_pub_->publish(odom_msg);
        
        // Publish TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom_msg.header;
        tf.child_frame_id = odom_msg.child_frame_id;
        tf.transform.translation.x = current_t_.x();
        tf.transform.translation.y = current_t_.y();
        tf.transform.translation.z = current_t_.z();
        tf.transform.rotation = odom_msg.pose.pose.orientation;
        
        tf_broadcaster_->sendTransform(tf);
    }
    
    // Core components
    std::unique_ptr<core::FeatureTracker> feature_tracker_;
    std::unique_ptr<core::StereoMatcher> stereo_matcher_;
    std::unique_ptr<core::PoseEstimator> pose_estimator_;
    std::unique_ptr<core::SlidingWindow> sliding_window_;
    std::unique_ptr<core::IMUPreintegrator> imu_preintegrator_;
    
    // ROS publishers/subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr quality_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracks_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_info_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>> sync_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // State
    bool use_imu_;
    bool initialized_;
    bool cam_params_set_ = false;
    int frame_id_;
    Eigen::Matrix3d current_R_;
    Eigen::Vector3d current_t_;
    std::vector<Eigen::Vector3d> prev_points_3d_;
    cv::Mat K_;
};

} // namespace ros
} // namespace stereo_vo

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereo_vo::ros::VONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
