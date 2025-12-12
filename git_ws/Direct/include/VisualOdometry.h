#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "DirectTypes.h"
#include "CameraModel.h"
#include "PointSelector.h"
#include "DirectTracker.h"
#include "DepthFilter.h"
#include "KeyframeManager.h"
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <vector>
#include <memory>

class VisualOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    VisualOdometry(const CalibrationParams& calib);
    
    // Ana işleme fonksiyonu
    bool ProcessFrame(const cv::Mat& image, double timestamp);
    
    // Trajectory'yi al
    std::vector<PoseData> GetTrajectory() const { return trajectory_; }
    
    // Mevcut pose'u al
    Sophus::SE3d GetCurrentPose() const { return current_pose_; }
    
    // İstatistikler
    void PrintStatistics() const;
    
    // Parametreler
    void SetVerbose(bool verbose) { verbose_ = verbose; }
    
private:
    // Modüller
    CameraModel cam_;
    std::unique_ptr<PointSelector> point_selector_;
    std::unique_ptr<DirectTracker> tracker_;
    std::unique_ptr<DepthFilter> depth_filter_;
    std::unique_ptr<KeyframeManager> keyframe_manager_;
    
    // Durum
    enum State {
        NOT_INITIALIZED,
        INITIALIZING,
        TRACKING,
        LOST
    };
    State state_;
    
    // Frame bilgisi
    cv::Mat current_frame_;
    cv::Mat last_frame_;
    double current_timestamp_;
    
    // Pose bilgisi
    Sophus::SE3d current_pose_;
    Sophus::SE3d last_pose_;
    
    // Motion model (constant velocity)
    Sophus::SE3d velocity_;
    
    // Noktalar
    std::vector<DirectPoint> current_points_;
    
    // Trajectory
    std::vector<PoseData> trajectory_;
    
    // Parametreler
    bool verbose_;
    int frame_count_;
    
    // İç fonksiyonlar
    bool Initialize();
    bool Track();
    void UpdateMotionModel();
    Sophus::SE3d PredictPose() const;
    void AddTrajectoryPoint();
};

#endif // VISUAL_ODOMETRY_H
