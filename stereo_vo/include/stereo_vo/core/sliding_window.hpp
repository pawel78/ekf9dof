/**
 * @file sliding_window.hpp
 * @brief Sliding window bundle adjustment using GTSAM
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <memory>

namespace stereo_vo {
namespace core {

/**
 * @brief Sliding window optimizer for visual odometry
 */
class SlidingWindow {
public:
    struct Config {
        int window_size = 7;                   ///< Number of frames in window
        double huber_threshold = 1.0;          ///< Huber loss threshold
        double chi_square_threshold = 5.991;   ///< Chi-square outlier threshold
        int max_iterations = 10;               ///< Max optimization iterations
        bool use_imu = false;                  ///< Whether to include IMU factors
    };

    struct Frame {
        int frame_id;
        Eigen::Matrix3d R;                     ///< Rotation
        Eigen::Vector3d t;                     ///< Translation
        std::vector<Eigen::Vector3d> points_3d;
        std::vector<cv::Point2f> observations;
        std::vector<int> track_ids;
    };

    explicit SlidingWindow(const Config& config = Config());
    ~SlidingWindow();

    /**
     * @brief Add a new frame to the sliding window
     * @param frame Frame with pose estimate and observations
     */
    void addFrame(const Frame& frame);

    /**
     * @brief Optimize the sliding window
     * @return Whether optimization succeeded
     */
    bool optimize();

    /**
     * @brief Get optimized pose for a frame
     * @param frame_id Frame identifier
     * @param R Output rotation
     * @param t Output translation
     * @return Whether frame was found
     */
    bool getPose(int frame_id, Eigen::Matrix3d& R, Eigen::Vector3d& t) const;

    /**
     * @brief Get optimized 3D points
     * @param track_id Track identifier
     * @param point Output 3D point
     * @return Whether point was found
     */
    bool getPoint(int track_id, Eigen::Vector3d& point) const;

    /**
     * @brief Clear the sliding window
     */
    void reset();

private:
    Config config_;
    std::deque<Frame> frames_;
    
#ifdef USE_GTSAM
    struct GTSAMImpl;
    std::unique_ptr<GTSAMImpl> gtsam_impl_;
#endif

    void maintainWindowSize();
};

} // namespace core
} // namespace stereo_vo
