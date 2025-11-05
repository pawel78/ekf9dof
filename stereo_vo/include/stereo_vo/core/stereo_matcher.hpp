/**
 * @file stereo_matcher.hpp
 * @brief Stereo matching using SGBM and triangulation
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <vector>

namespace stereo_vo {
namespace core {

/**
 * @brief Perform stereo matching and 3D point triangulation
 */
class StereoMatcher {
public:
    struct Config {
        std::string method = "sgbm";      ///< "sgbm" or "cuda_sgbm"
        int min_disparity = 0;
        int num_disparities = 128;        ///< Must be divisible by 16
        int block_size = 5;
        int p1 = 8 * 3 * 5 * 5;          ///< Penalty for disparity change by ±1
        int p2 = 32 * 3 * 5 * 5;         ///< Penalty for larger disparity changes
        int disp12_max_diff = 1;
        int uniqueness_ratio = 10;
        int speckle_window_size = 100;
        int speckle_range = 32;
        int mode = cv::StereoSGBM::MODE_SGBM_3WAY;
    };

    struct CameraParams {
        cv::Mat K_left;                   ///< Left camera intrinsics (3x3)
        cv::Mat K_right;                  ///< Right camera intrinsics (3x3)
        cv::Mat D_left;                   ///< Left distortion coefficients
        cv::Mat D_right;                  ///< Right distortion coefficients
        cv::Mat R;                        ///< Rotation from left to right
        cv::Mat T;                        ///< Translation from left to right
        double baseline;                  ///< Baseline in meters
    };

    explicit StereoMatcher(const Config& config = Config());
    ~StereoMatcher() = default;

    /**
     * @brief Initialize with camera calibration parameters
     */
    void setCameraParams(const CameraParams& params);

    /**
     * @brief Compute disparity map from rectified stereo pair
     * @param left_rect Rectified left image
     * @param right_rect Rectified right image
     * @param disparity Output disparity map
     */
    void computeDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect, 
                         cv::Mat& disparity);

    /**
     * @brief Match stereo correspondences for given features
     * @param features_left Features in left image
     * @param right_image Right grayscale image
     * @param features_right Output matched features in right image
     * @param mask Output mask indicating successful matches
     */
    void matchFeatures(const std::vector<cv::Point2f>& features_left,
                      const cv::Mat& right_image,
                      std::vector<cv::Point2f>& features_right,
                      std::vector<bool>& mask);

    /**
     * @brief Triangulate 3D points from stereo correspondences
     * @param pts_left Feature points in left image
     * @param pts_right Corresponding points in right image
     * @param points_3d Output 3D points in left camera frame
     */
    void triangulate(const std::vector<cv::Point2f>& pts_left,
                    const std::vector<cv::Point2f>& pts_right,
                    std::vector<Eigen::Vector3d>& points_3d);

private:
    Config config_;
    CameraParams cam_params_;
    cv::Ptr<cv::StereoSGBM> sgbm_;
    bool params_initialized_;

    void initSGBM();
};

} // namespace core
} // namespace stereo_vo
