/**
 * @file test_pose_estimator.cpp
 * @brief Unit tests for pose estimator
 */

#include <gtest/gtest.h>
#include "stereo_vo/core/pose_estimator.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace stereo_vo::core;

class PoseEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple camera matrix
        K_ = (cv::Mat_<double>(3, 3) << 
            500, 0, 320,
            0, 500, 240,
            0, 0, 1);
        
        // Create some 3D points
        points_3d_.push_back(Eigen::Vector3d(0, 0, 2));
        points_3d_.push_back(Eigen::Vector3d(1, 0, 2));
        points_3d_.push_back(Eigen::Vector3d(0, 1, 2));
        points_3d_.push_back(Eigen::Vector3d(1, 1, 2));
        points_3d_.push_back(Eigen::Vector3d(-1, 0, 2));
        points_3d_.push_back(Eigen::Vector3d(0, -1, 2));
        
        // Project to 2D
        for (const auto& pt3d : points_3d_) {
            double u = K_.at<double>(0, 0) * pt3d.x() / pt3d.z() + K_.at<double>(0, 2);
            double v = K_.at<double>(1, 1) * pt3d.y() / pt3d.z() + K_.at<double>(1, 2);
            points_2d_.push_back(cv::Point2f(u, v));
        }
    }
    
    cv::Mat K_;
    std::vector<Eigen::Vector3d> points_3d_;
    std::vector<cv::Point2f> points_2d_;
};

TEST_F(PoseEstimatorTest, Initialization) {
    PoseEstimator::Config config;
    PoseEstimator estimator(config);
    // Should construct without error
}

TEST_F(PoseEstimatorTest, IdentityPose) {
    PoseEstimator::Config config;
    config.min_inliers = 4;
    PoseEstimator estimator(config);
    
    PoseEstimator::PoseResult result;
    estimator.estimatePose(points_3d_, points_2d_, K_, result);
    
    EXPECT_TRUE(result.success);
    EXPECT_GE(result.num_inliers, 4);
    
    // Should be close to identity
    Eigen::Matrix3d expected_R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d expected_t = Eigen::Vector3d::Zero();
    
    // Check rotation is close to identity
    double rot_error = (result.R - expected_R).norm();
    EXPECT_LT(rot_error, 0.1);
    
    // Check translation is small
    EXPECT_LT(result.t.norm(), 0.5);
}

TEST_F(PoseEstimatorTest, InsufficientPoints) {
    PoseEstimator::Config config;
    PoseEstimator estimator(config);
    
    std::vector<Eigen::Vector3d> few_3d = {points_3d_[0], points_3d_[1]};
    std::vector<cv::Point2f> few_2d = {points_2d_[0], points_2d_[1]};
    
    PoseEstimator::PoseResult result;
    estimator.estimatePose(few_3d, few_2d, K_, result);
    
    EXPECT_FALSE(result.success);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
