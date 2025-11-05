/**
 * @file test_feature_tracker.cpp
 * @brief Unit tests for feature tracker
 */

#include <gtest/gtest.h>
#include "stereo_vo/core/feature_tracker.hpp"
#include <opencv2/opencv.hpp>

using namespace stereo_vo::core;

class FeatureTrackerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple test image with corners
        test_image_ = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::rectangle(test_image_, cv::Point(100, 100), cv::Point(200, 200), 255, 2);
        cv::rectangle(test_image_, cv::Point(300, 150), cv::Point(400, 250), 255, 2);
        cv::circle(test_image_, cv::Point(500, 300), 50, 255, 2);
    }
    
    cv::Mat test_image_;
};

TEST_F(FeatureTrackerTest, Initialization) {
    FeatureTracker::Config config;
    config.max_features = 100;
    FeatureTracker tracker(config);
    
    EXPECT_EQ(tracker.getNumTrackedFeatures(), 0);
}

TEST_F(FeatureTrackerTest, DetectFeatures) {
    FeatureTracker::Config config;
    config.max_features = 100;
    FeatureTracker tracker(config);
    
    std::vector<FeatureTracker::Feature> features;
    tracker.processFrame(test_image_, features);
    
    EXPECT_GT(features.size(), 0);
    EXPECT_LE(features.size(), 100);
}

TEST_F(FeatureTrackerTest, TrackFeatures) {
    FeatureTracker::Config config;
    config.max_features = 100;
    FeatureTracker tracker(config);
    
    // First frame
    std::vector<FeatureTracker::Feature> features1;
    tracker.processFrame(test_image_, features1);
    size_t initial_count = features1.size();
    
    // Second frame (same image)
    std::vector<FeatureTracker::Feature> features2;
    tracker.processFrame(test_image_, features2);
    
    // Should track most features
    EXPECT_GT(features2.size(), initial_count * 0.5);
}

TEST_F(FeatureTrackerTest, Reset) {
    FeatureTracker::Config config;
    FeatureTracker tracker(config);
    
    std::vector<FeatureTracker::Feature> features;
    tracker.processFrame(test_image_, features);
    EXPECT_GT(features.size(), 0);
    
    tracker.reset();
    EXPECT_EQ(tracker.getNumTrackedFeatures(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
