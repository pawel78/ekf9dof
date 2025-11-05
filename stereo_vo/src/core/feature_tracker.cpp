/**
 * @file feature_tracker.cpp
 * @brief Implementation of feature tracking
 */

#include "stereo_vo/core/feature_tracker.hpp"
#include <opencv2/video/tracking.hpp>
#include <algorithm>

namespace stereo_vo {
namespace core {

FeatureTracker::FeatureTracker(const Config& config)
    : config_(config), next_track_id_(0) {
    // Create ORB detector
    orb_detector_ = cv::ORB::create(config_.max_features);
}

void FeatureTracker::processFrame(const cv::Mat& image, std::vector<Feature>& features) {
    if (prev_image_.empty()) {
        // First frame: detect features
        std::vector<cv::Point2f> new_pts;
        detectNewFeatures(image, new_pts);
        
        current_features_.clear();
        for (const auto& pt : new_pts) {
            Feature feat;
            feat.pt = pt;
            feat.prev_pt = pt;
            feat.track_id = next_track_id_++;
            feat.lifetime = 1;
            feat.quality = 1.0f;
            feat.matched_stereo = false;
            current_features_.push_back(feat);
        }
    } else {
        // Track existing features
        trackFeatures(image);
        
        // Detect new features if needed
        if (current_features_.size() < static_cast<size_t>(config_.max_features * 0.7)) {
            std::vector<cv::Point2f> new_pts;
            detectNewFeatures(image, new_pts);
            
            for (const auto& pt : new_pts) {
                Feature feat;
                feat.pt = pt;
                feat.prev_pt = pt;
                feat.track_id = next_track_id_++;
                feat.lifetime = 1;
                feat.quality = 1.0f;
                feat.matched_stereo = false;
                current_features_.push_back(feat);
            }
        }
    }
    
    // Cull poor quality features
    cullFeatures();
    
    prev_image_ = image.clone();
    features = current_features_;
}

void FeatureTracker::detectNewFeatures(const cv::Mat& image, std::vector<cv::Point2f>& new_pts) {
    // Create mask to avoid existing features
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8U) * 255;
    for (const auto& feat : current_features_) {
        cv::circle(mask, feat.pt, static_cast<int>(config_.min_feature_distance), 0, -1);
    }
    
    // Detect corners using Shi-Tomasi
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(image, corners, config_.max_features - current_features_.size(),
                           config_.quality_level, config_.min_feature_distance, mask);
    
    new_pts = corners;
}

void FeatureTracker::trackFeatures(const cv::Mat& image) {
    if (current_features_.empty()) return;
    
    // Extract previous and current points
    std::vector<cv::Point2f> prev_pts, curr_pts;
    for (const auto& feat : current_features_) {
        prev_pts.push_back(feat.pt);
    }
    
    // LK optical flow tracking
    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
    cv::Size win_size(config_.lk_win_size, config_.lk_win_size);
    
    cv::calcOpticalFlowPyrLK(prev_image_, image, prev_pts, curr_pts, status, err,
                            win_size, config_.lk_levels, criteria);
    
    // Update features with tracked points
    std::vector<Feature> tracked_features;
    for (size_t i = 0; i < current_features_.size(); ++i) {
        if (status[i] && curr_pts[i].x >= 0 && curr_pts[i].y >= 0 &&
            curr_pts[i].x < image.cols && curr_pts[i].y < image.rows) {
            Feature feat = current_features_[i];
            feat.prev_pt = feat.pt;
            feat.pt = curr_pts[i];
            feat.lifetime++;
            feat.quality = 1.0f / (1.0f + err[i]);
            tracked_features.push_back(feat);
        }
    }
    
    current_features_ = tracked_features;
}

void FeatureTracker::rejectOutliers(std::vector<Feature>& features, const cv::Mat& F) {
    if (features.size() < 8 || F.empty()) return;
    
    std::vector<Feature> inliers;
    for (const auto& feat : features) {
        // Compute epipolar error: x'^T * F * x
        cv::Mat x1 = (cv::Mat_<double>(3, 1) << feat.prev_pt.x, feat.prev_pt.y, 1.0);
        cv::Mat x2 = (cv::Mat_<double>(3, 1) << feat.pt.x, feat.pt.y, 1.0);
        cv::Mat error = x2.t() * F * x1;
        double epipolar_error = std::abs(error.at<double>(0));
        
        if (epipolar_error < config_.epipolar_thresh_px) {
            inliers.push_back(feat);
        }
    }
    
    features = inliers;
}

void FeatureTracker::cullFeatures() {
    // Remove features with low quality or too short lifetime
    std::vector<Feature> culled;
    for (const auto& feat : current_features_) {
        if (feat.quality > 0.1f && feat.lifetime >= 2) {
            culled.push_back(feat);
        }
    }
    current_features_ = culled;
}

void FeatureTracker::reset() {
    prev_image_.release();
    current_features_.clear();
    next_track_id_ = 0;
}

} // namespace core
} // namespace stereo_vo
