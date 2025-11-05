/**
 * @file sliding_window.cpp
 * @brief Implementation of sliding window bundle adjustment
 */

#include "stereo_vo/core/sliding_window.hpp"

#ifdef USE_GTSAM
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#endif

namespace stereo_vo {
namespace core {

#ifdef USE_GTSAM
struct SlidingWindow::GTSAMImpl {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;
    gtsam::Values optimized_values;
};
#endif

SlidingWindow::SlidingWindow(const Config& config)
    : config_(config) {
#ifdef USE_GTSAM
    gtsam_impl_ = std::make_unique<GTSAMImpl>();
#endif
}

SlidingWindow::~SlidingWindow() = default;

void SlidingWindow::addFrame(const Frame& frame) {
    frames_.push_back(frame);
    maintainWindowSize();
}

bool SlidingWindow::optimize() {
#ifdef USE_GTSAM
    if (frames_.empty()) return false;
    
    gtsam_impl_->graph.resize(0);
    gtsam_impl_->initial_estimate.clear();
    
    // Add poses to initial estimate
    for (size_t i = 0; i < frames_.size(); ++i) {
        const auto& frame = frames_[i];
        gtsam::Rot3 R(frame.R);
        gtsam::Point3 t(frame.t);
        gtsam::Pose3 pose(R, t);
        gtsam_impl_->initial_estimate.insert(gtsam::Symbol('x', frame.frame_id), pose);
    }
    
    // Add landmark points
    std::map<int, Eigen::Vector3d> landmarks;
    for (const auto& frame : frames_) {
        for (size_t i = 0; i < frame.track_ids.size(); ++i) {
            if (i < frame.points_3d.size()) {
                landmarks[frame.track_ids[i]] = frame.points_3d[i];
            }
        }
    }
    
    for (const auto& [track_id, point] : landmarks) {
        gtsam::Point3 pt(point.x(), point.y(), point.z());
        gtsam_impl_->initial_estimate.insert(gtsam::Symbol('l', track_id), pt);
    }
    
    // Optimize
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = config_.max_iterations;
    gtsam::LevenbergMarquardtOptimizer optimizer(gtsam_impl_->graph, gtsam_impl_->initial_estimate, params);
    gtsam_impl_->optimized_values = optimizer.optimize();
    
    return true;
#else
    // Without GTSAM, just use initial estimates
    return !frames_.empty();
#endif
}

bool SlidingWindow::getPose(int frame_id, Eigen::Matrix3d& R, Eigen::Vector3d& t) const {
#ifdef USE_GTSAM
    try {
        auto pose = gtsam_impl_->optimized_values.at<gtsam::Pose3>(gtsam::Symbol('x', frame_id));
        auto rot = pose.rotation().matrix();
        R = rot;
        t = pose.translation();
        return true;
    } catch (...) {
        // Fall through to find in frames
    }
#endif
    
    // Search in frames
    for (const auto& frame : frames_) {
        if (frame.frame_id == frame_id) {
            R = frame.R;
            t = frame.t;
            return true;
        }
    }
    return false;
}

bool SlidingWindow::getPoint(int track_id, Eigen::Vector3d& point) const {
#ifdef USE_GTSAM
    try {
        auto pt = gtsam_impl_->optimized_values.at<gtsam::Point3>(gtsam::Symbol('l', track_id));
        point = Eigen::Vector3d(pt.x(), pt.y(), pt.z());
        return true;
    } catch (...) {
        return false;
    }
#else
    return false;
#endif
}

void SlidingWindow::reset() {
    frames_.clear();
#ifdef USE_GTSAM
    gtsam_impl_->graph.resize(0);
    gtsam_impl_->initial_estimate.clear();
    gtsam_impl_->optimized_values.clear();
#endif
}

void SlidingWindow::maintainWindowSize() {
    while (frames_.size() > static_cast<size_t>(config_.window_size)) {
        frames_.pop_front();
    }
}

} // namespace core
} // namespace stereo_vo
