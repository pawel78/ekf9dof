/**
 * @file pose_estimator.cpp
 * @brief Implementation of pose estimation
 */

#include "stereo_vo/core/pose_estimator.hpp"
#include <opencv2/calib3d.hpp>

namespace stereo_vo {
namespace core {

PoseEstimator::PoseEstimator(const Config& config)
    : config_(config) {
}

void PoseEstimator::estimatePose(const std::vector<Eigen::Vector3d>& points_3d,
                                const std::vector<cv::Point2f>& points_2d,
                                const cv::Mat& K,
                                PoseResult& result) {
    result.success = false;
    
    if (points_3d.size() < static_cast<size_t>(config_.min_inliers) ||
        points_3d.size() != points_2d.size()) {
        return;
    }
    
    // Convert to OpenCV format
    std::vector<cv::Point3d> object_points;
    for (const auto& pt : points_3d) {
        object_points.emplace_back(pt.x(), pt.y(), pt.z());
    }
    
    // Solve PnP with RANSAC
    cv::Mat rvec, tvec, inliers;
    bool success = cv::solvePnPRansac(
        object_points, points_2d, K, cv::Mat(),
        rvec, tvec, false,
        config_.ransac_iterations,
        config_.ransac_reproj_thresh_px,
        config_.ransac_confidence,
        inliers,
        cv::SOLVEPNP_EPNP
    );
    
    if (!success || inliers.rows < config_.min_inliers) {
        return;
    }
    
    // Convert rotation vector to matrix
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    
    // Convert to Eigen
    result.R << R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2);
    
    result.t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    
    // Extract inlier indices
    result.inlier_indices.clear();
    for (int i = 0; i < inliers.rows; ++i) {
        result.inlier_indices.push_back(inliers.at<int>(i));
    }
    result.num_inliers = inliers.rows;
    
    // Compute reprojection error
    double total_error = 0.0;
    for (int idx : result.inlier_indices) {
        const auto& pt3d = points_3d[idx];
        const auto& pt2d = points_2d[idx];
        
        // Project 3D point
        Eigen::Vector3d proj = result.R * pt3d + result.t;
        double u = K.at<double>(0,0) * proj.x() / proj.z() + K.at<double>(0,2);
        double v = K.at<double>(1,1) * proj.y() / proj.z() + K.at<double>(1,2);
        
        double error = std::sqrt(std::pow(u - pt2d.x, 2) + std::pow(v - pt2d.y, 2));
        total_error += error;
    }
    result.reprojection_error = total_error / result.num_inliers;
    
    // Optionally refine with LM
    if (config_.refine_with_lm) {
        refinePose(points_3d, points_2d, K, result.inlier_indices, result.R, result.t);
    }
    
    result.success = true;
}

int PoseEstimator::estimateEssential(const std::vector<cv::Point2f>& pts1,
                                     const std::vector<cv::Point2f>& pts2,
                                     const cv::Mat& K,
                                     cv::Mat& E,
                                     std::vector<bool>& inlier_mask) {
    if (pts1.size() < 8 || pts1.size() != pts2.size()) {
        return 0;
    }
    
    cv::Mat mask;
    E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC,
                            config_.ransac_confidence,
                            config_.ransac_reproj_thresh_px,
                            mask);
    
    // Convert mask to bool vector
    inlier_mask.clear();
    int num_inliers = 0;
    for (int i = 0; i < mask.rows; ++i) {
        bool is_inlier = mask.at<uchar>(i) > 0;
        inlier_mask.push_back(is_inlier);
        if (is_inlier) num_inliers++;
    }
    
    return num_inliers;
}

void PoseEstimator::decomposeEssential(const cv::Mat& E,
                                      const std::vector<cv::Point2f>& pts1,
                                      const std::vector<cv::Point2f>& pts2,
                                      const cv::Mat& K,
                                      Eigen::Matrix3d& R,
                                      Eigen::Vector3d& t) {
    cv::Mat R_cv, t_cv;
    cv::Mat mask;
    cv::recoverPose(E, pts1, pts2, K, R_cv, t_cv, mask);
    
    // Convert to Eigen
    R << R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
         R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
         R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2);
    
    t << t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2);
}

void PoseEstimator::refinePose(const std::vector<Eigen::Vector3d>& points_3d,
                              const std::vector<cv::Point2f>& points_2d,
                              const cv::Mat& K,
                              const std::vector<int>& inliers,
                              Eigen::Matrix3d& R,
                              Eigen::Vector3d& t) {
    // Extract inlier correspondences
    std::vector<cv::Point3d> obj_pts;
    std::vector<cv::Point2f> img_pts;
    for (int idx : inliers) {
        obj_pts.emplace_back(points_3d[idx].x(), points_3d[idx].y(), points_3d[idx].z());
        img_pts.push_back(points_2d[idx]);
    }
    
    // Convert to cv format
    cv::Mat R_cv = (cv::Mat_<double>(3,3) <<
        R(0,0), R(0,1), R(0,2),
        R(1,0), R(1,1), R(1,2),
        R(2,0), R(2,1), R(2,2));
    cv::Mat t_cv = (cv::Mat_<double>(3,1) << t.x(), t.y(), t.z());
    cv::Mat rvec;
    cv::Rodrigues(R_cv, rvec);
    
    // Refine with iterative PnP
    cv::solvePnPRefineLM(obj_pts, img_pts, K, cv::Mat(), rvec, t_cv);
    
    // Convert back
    cv::Rodrigues(rvec, R_cv);
    R << R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
         R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
         R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2);
    t << t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2);
}

} // namespace core
} // namespace stereo_vo
