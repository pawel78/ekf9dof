/**
 * @file pose_estimator.hpp
 * @brief Pose estimation using PnP and RANSAC
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>

namespace stereo_vo {
namespace core {

/**
 * @brief Estimate camera pose from 2D-3D correspondences
 */
class PoseEstimator {
public:
    struct Config {
        double ransac_reproj_thresh_px = 2.0;  ///< RANSAC reprojection threshold
        int ransac_iterations = 100;           ///< RANSAC max iterations
        double ransac_confidence = 0.99;       ///< RANSAC confidence level
        int min_inliers = 20;                  ///< Minimum inliers for valid pose
        bool refine_with_lm = true;            ///< Refine with Levenberg-Marquardt
    };

    struct PoseResult {
        Eigen::Matrix3d R;                     ///< Rotation matrix
        Eigen::Vector3d t;                     ///< Translation vector
        std::vector<int> inlier_indices;       ///< Indices of inlier correspondences
        int num_inliers;                       ///< Number of inliers
        double reprojection_error;             ///< Mean reprojection error
        bool success;                          ///< Whether estimation succeeded
    };

    explicit PoseEstimator(const Config& config = Config());
    ~PoseEstimator() = default;

    /**
     * @brief Estimate pose using PnP with RANSAC
     * @param points_3d 3D points in world/previous camera frame
     * @param points_2d Corresponding 2D observations in current frame
     * @param K Camera intrinsic matrix
     * @param result Output pose and statistics
     */
    void estimatePose(const std::vector<Eigen::Vector3d>& points_3d,
                     const std::vector<cv::Point2f>& points_2d,
                     const cv::Mat& K,
                     PoseResult& result);

    /**
     * @brief Estimate essential matrix from point correspondences
     * @param pts1 Points in first image
     * @param pts2 Points in second image
     * @param K Camera intrinsic matrix
     * @param E Output essential matrix
     * @param inlier_mask Output inlier mask
     * @return Number of inliers
     */
    int estimateEssential(const std::vector<cv::Point2f>& pts1,
                         const std::vector<cv::Point2f>& pts2,
                         const cv::Mat& K,
                         cv::Mat& E,
                         std::vector<bool>& inlier_mask);

    /**
     * @brief Decompose essential matrix to R and t
     * @param E Essential matrix
     * @param pts1 Points in first image
     * @param pts2 Points in second image
     * @param K Camera intrinsics
     * @param R Output rotation
     * @param t Output translation (unit vector)
     */
    void decomposeEssential(const cv::Mat& E,
                           const std::vector<cv::Point2f>& pts1,
                           const std::vector<cv::Point2f>& pts2,
                           const cv::Mat& K,
                           Eigen::Matrix3d& R,
                           Eigen::Vector3d& t);

private:
    Config config_;

    void refinePose(const std::vector<Eigen::Vector3d>& points_3d,
                   const std::vector<cv::Point2f>& points_2d,
                   const cv::Mat& K,
                   const std::vector<int>& inliers,
                   Eigen::Matrix3d& R,
                   Eigen::Vector3d& t);
};

} // namespace core
} // namespace stereo_vo
