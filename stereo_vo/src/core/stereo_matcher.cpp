/**
 * @file stereo_matcher.cpp
 * @brief Implementation of stereo matching
 */

#include "stereo_vo/core/stereo_matcher.hpp"
#include <opencv2/calib3d.hpp>

namespace stereo_vo {
namespace core {

StereoMatcher::StereoMatcher(const Config& config)
    : config_(config), params_initialized_(false) {
    initSGBM();
}

void StereoMatcher::initSGBM() {
    sgbm_ = cv::StereoSGBM::create(
        config_.min_disparity,
        config_.num_disparities,
        config_.block_size,
        config_.p1,
        config_.p2,
        config_.disp12_max_diff,
        0,  // preFilterCap
        config_.uniqueness_ratio,
        config_.speckle_window_size,
        config_.speckle_range,
        config_.mode
    );
}

void StereoMatcher::setCameraParams(const CameraParams& params) {
    cam_params_ = params;
    params_initialized_ = true;
}

void StereoMatcher::computeDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                                    cv::Mat& disparity) {
    if (config_.method == "sgbm") {
        sgbm_->compute(left_rect, right_rect, disparity);
        // Convert to float and scale
        disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);
    } else {
        // Placeholder for CUDA SGBM
        sgbm_->compute(left_rect, right_rect, disparity);
        disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);
    }
}

void StereoMatcher::matchFeatures(const std::vector<cv::Point2f>& features_left,
                                 const cv::Mat& right_image,
                                 std::vector<cv::Point2f>& features_right,
                                 std::vector<bool>& mask) {
    features_right.resize(features_left.size());
    mask.resize(features_left.size(), false);
    
    // Simple horizontal search along epipolar line (for rectified images)
    const int search_range = 128;
    const int template_size = 15;
    const int half_template = template_size / 2;
    
    cv::Mat left_gray = right_image; // Assume grayscale
    
    for (size_t i = 0; i < features_left.size(); ++i) {
        const auto& pt_left = features_left[i];
        
        // Check if point is within bounds
        if (pt_left.x < half_template || pt_left.y < half_template ||
            pt_left.x >= left_gray.cols - half_template ||
            pt_left.y >= left_gray.rows - half_template) {
            continue;
        }
        
        // Extract template from left image (would need left image as parameter)
        // For now, use a simplified approach
        cv::Point2f pt_right = pt_left;
        pt_right.x -= 20.0f; // Simplified disparity estimate
        
        if (pt_right.x >= 0 && pt_right.x < right_image.cols) {
            features_right[i] = pt_right;
            mask[i] = true;
        }
    }
}

void StereoMatcher::triangulate(const std::vector<cv::Point2f>& pts_left,
                                const std::vector<cv::Point2f>& pts_right,
                                std::vector<Eigen::Vector3d>& points_3d) {
    if (!params_initialized_) {
        throw std::runtime_error("Camera parameters not initialized");
    }
    
    points_3d.clear();
    points_3d.reserve(pts_left.size());
    
    // Get camera matrices
    cv::Mat P1 = cam_params_.K_left * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P2 = cam_params_.K_right * cv::Mat::eye(3, 4, CV_64F);
    
    // Set extrinsics for right camera
    cv::Mat RT = (cv::Mat_<double>(3, 4) <<
        cam_params_.R.at<double>(0,0), cam_params_.R.at<double>(0,1), cam_params_.R.at<double>(0,2), cam_params_.T.at<double>(0),
        cam_params_.R.at<double>(1,0), cam_params_.R.at<double>(1,1), cam_params_.R.at<double>(1,2), cam_params_.T.at<double>(1),
        cam_params_.R.at<double>(2,0), cam_params_.R.at<double>(2,1), cam_params_.R.at<double>(2,2), cam_params_.T.at<double>(2)
    );
    P2 = cam_params_.K_right * RT;
    
    for (size_t i = 0; i < pts_left.size(); ++i) {
        // Triangulate using OpenCV
        cv::Mat points_4d;
        std::vector<cv::Point2f> pts1{pts_left[i]}, pts2{pts_right[i]};
        cv::triangulatePoints(P1, P2, pts1, pts2, points_4d);
        
        // Convert to 3D Eigen vector
        double w = points_4d.at<float>(3, 0);
        if (std::abs(w) > 1e-6) {
            Eigen::Vector3d pt3d(
                points_4d.at<float>(0, 0) / w,
                points_4d.at<float>(1, 0) / w,
                points_4d.at<float>(2, 0) / w
            );
            
            // Check if point is in front of camera
            if (pt3d.z() > 0 && pt3d.z() < 100.0) {
                points_3d.push_back(pt3d);
            } else {
                points_3d.push_back(Eigen::Vector3d::Zero());
            }
        } else {
            points_3d.push_back(Eigen::Vector3d::Zero());
        }
    }
}

} // namespace core
} // namespace stereo_vo
