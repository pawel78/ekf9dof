/**
 * @file feature_tracker.hpp
 * @brief ORB feature detection and pyramidal LK optical flow tracking
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace stereo_vo {
namespace core {

/**
 * @brief Track features across frames using ORB detection and LK optical flow
 */
class FeatureTracker {
public:
    struct Config {
        int max_features = 1500;           ///< Maximum number of features to track
        int lk_levels = 3;                 ///< Pyramid levels for LK tracking
        int lk_win_size = 21;              ///< LK window size
        double min_feature_distance = 15.0; ///< Minimum pixel distance between features
        double quality_level = 0.01;        ///< Shi-Tomasi corner quality
        double epipolar_thresh_px = 2.0;   ///< Epipolar constraint threshold
    };

    struct Feature {
        cv::Point2f pt;           ///< Current pixel location
        cv::Point2f prev_pt;      ///< Previous pixel location
        int track_id;             ///< Unique track identifier
        int lifetime;             ///< Number of frames tracked
        float quality;            ///< Feature quality score
        bool matched_stereo;      ///< Whether feature has stereo correspondence
    };

    explicit FeatureTracker(const Config& config = Config());
    ~FeatureTracker() = default;

    /**
     * @brief Process a new frame, track existing features and detect new ones
     * @param image Current grayscale image
     * @param features Output tracked features
     */
    void processFrame(const cv::Mat& image, std::vector<Feature>& features);

    /**
     * @brief Reject outliers based on epipolar constraint
     * @param features Input/output features (outliers removed)
     * @param F Fundamental matrix
     */
    void rejectOutliers(std::vector<Feature>& features, const cv::Mat& F);

    /**
     * @brief Get current number of tracked features
     */
    size_t getNumTrackedFeatures() const { return current_features_.size(); }

    /**
     * @brief Reset tracker state
     */
    void reset();

private:
    Config config_;
    cv::Mat prev_image_;
    std::vector<Feature> current_features_;
    int next_track_id_;
    cv::Ptr<cv::ORB> orb_detector_;

    void detectNewFeatures(const cv::Mat& image, std::vector<cv::Point2f>& new_pts);
    void trackFeatures(const cv::Mat& image);
    void cullFeatures();
};

} // namespace core
} // namespace stereo_vo
