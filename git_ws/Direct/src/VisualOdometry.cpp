#include "VisualOdometry.h"
#include <iostream>

VisualOdometry::VisualOdometry(const CalibrationParams& calib)
    : cam_(calib),
      state_(NOT_INITIALIZED),
      current_timestamp_(0),
      verbose_(true),
      frame_count_(0) {
    
    point_selector_ = std::make_unique<PointSelector>(cam_);
    tracker_ = std::make_unique<DirectTracker>(cam_);
    depth_filter_ = std::make_unique<DepthFilter>(cam_);
    keyframe_manager_ = std::make_unique<KeyframeManager>();
    
    current_pose_ = Sophus::SE3d();
    last_pose_ = Sophus::SE3d();
    velocity_ = Sophus::SE3d();
    
    // Tracker parametreleri - optimize for speed
    tracker_->SetPyramidLevels(3);      // Coarse-to-fine için 3 level (rotation için önemli)
    tracker_->SetMaxIterations(8);      // Rotation için biraz daha iterasyon
    tracker_->SetHuberThreshold(15.0);  // Daha toleranslı (texture-less için)
}

bool VisualOdometry::ProcessFrame(const cv::Mat& image, double timestamp) {
    frame_count_++;
    current_timestamp_ = timestamp;
    
    // Gri tonlamaya çevir
    if (image.channels() == 3) {
        cv::cvtColor(image, current_frame_, cv::COLOR_BGR2GRAY);
    } else {
        current_frame_ = image.clone();
    }
    
    bool success = false;
    
    switch (state_) {
        case NOT_INITIALIZED:
            success = Initialize();
            break;
            
        case INITIALIZING:
            success = Initialize();
            break;
            
        case TRACKING:
            success = Track();
            break;
            
        case LOST:
            if (verbose_) {
                std::cout << "Tracking lost! Trying to reinitialize..." << std::endl;
            }
            state_ = NOT_INITIALIZED;
            success = Initialize();
            break;
    }
    
    if (success) {
        AddTrajectoryPoint();
    }
    
    // Frame'i sakla
    last_frame_ = current_frame_.clone();
    last_pose_ = current_pose_;
    
    return success;
}

bool VisualOdometry::Initialize() {
    if (verbose_) {
        std::cout << "\n=== Initializing (Frame " << frame_count_ << ") ===" << std::endl;
    }
    
    // İlk frame için nokta seç - DSO tarzı sparse (400-600 nokta)
    // Daha fazla nokta = rotation için daha robust
    current_points_ = point_selector_->SelectPointsGrid(current_frame_, 25, 4);  // ~500 nokta
    
    if (current_points_.size() < 100) {  // Minimum 100 nokta (rotation için)
        if (verbose_) {
            std::cout << "Not enough points: " << current_points_.size() << std::endl;
        }
        return false;
    }
    
    // Depth'leri initialize et - SABİT DEPTH (hız için)
    // Depth filtering kapalı, ama kod duruyor
    depth_filter_->InitializeDepths(current_points_, 2.5f);  // Varsayılan 2.5 metre
    
    /* DEPTH FİLTERİNG - İsterseniz açabilirsiniz (yavaşlatır)
    // İlk depth estimation için epipolar search yapabilir
    if (frame_count_ > 5) {
        depth_filter_->UpdateDepths(current_points_, last_frame_, current_frame_, 
                                   last_pose_, current_pose_);
    }
    */
    
    // İlk keyframe ekle
    current_pose_ = Sophus::SE3d();
    keyframe_manager_->AddKeyframe(
        current_timestamp_,
        current_frame_,
        current_pose_,
        current_points_
    );
    
    state_ = TRACKING;
    
    if (verbose_) {
        std::cout << "Initialized with " << current_points_.size() << " points" << std::endl;
    }
    
    return true;
}

bool VisualOdometry::Track() {
    if (verbose_) {
        std::cout << "\n=== Tracking (Frame " << frame_count_ << ") ===" << std::endl;
    }
    
    auto last_kf = keyframe_manager_->GetLastKeyframe();
    if (!last_kf) {
        state_ = NOT_INITIALIZED;
        return false;
    }
    
    // Motion model ile pose tahmini (translation + rotation)
    Sophus::SE3d predicted_pose = PredictPose();
    
    // Eğer hareket çok büyükse, son pose'u kullan (safety)
    double motion_magnitude = predicted_pose.translation().norm();
    if (motion_magnitude > 1.0) {  // 1 metre'den fazla hareket şüpheli
        predicted_pose = last_pose_;  // Son pose'dan başla
    }
    
    // Tracking
    Sophus::SE3d tracked_pose = predicted_pose;
    bool track_success = tracker_->Track(
        last_kf->image,
        current_frame_,
        current_points_,
        tracked_pose,
        predicted_pose
    );
    
    if (!track_success) {
        if (verbose_) {
            std::cout << "Tracking failed!" << std::endl;
        }
        state_ = LOST;
        return false;
    }
    
    current_pose_ = tracked_pose;
    
    // İstatistikler
    int num_good_points = 0;
    for (const auto& pt : current_points_) {
        if (pt.is_good) num_good_points++;
    }
    
    if (verbose_) {
        std::cout << "Tracked points: " << num_good_points 
                  << "/" << current_points_.size() << std::endl;
        std::cout << "Average error: " << tracker_->GetLastError() << std::endl;
        
        Eigen::Vector3d trans = current_pose_.translation();
        std::cout << "Position: [" << trans.transpose() << "]" << std::endl;
    }
    
    // Çok az nokta kaldıysa
    if (num_good_points < 50) {
        if (verbose_) {
            std::cout << "Too few tracked points!" << std::endl;
        }
        state_ = LOST;
        return false;
    }
    
    /* DEPTH GÜNCELLEME - Hız için kapalı, gerekirse açabilirsiniz
    // Depth filtering aktif edersek, keyframe'lerde depth güncelle
    // Bu LSD-SLAM/DSO tarzı depth estimation'dır
    if (frame_count_ % 10 == 0) {  // Her 10 frame'de bir (çok yavaş!)
        depth_filter_->UpdateDepths(
            current_points_,
            last_kf->image,
            current_frame_,
            last_kf->pose,
            current_pose_
        );
    }
    // Alternatif: Stereo depth veya RGB-D kullanabilirsiniz
    // Alternatif: Triangulation ile depth hesaplayabilirsiniz
    */
    
    // Keyframe kontrolü
    if (keyframe_manager_->NeedNewKeyframe(current_pose_, num_good_points)) {
        // Yeni noktalar ekle - rotation tracking için daha fazla nokta
        auto new_points = point_selector_->SelectPointsGrid(current_frame_, 25, 4);  // ~500 nokta
        depth_filter_->InitializeDepths(new_points, 2.5f);  // Sabit depth varsayımı
        
        // Eski iyi noktaları koru
        for (const auto& pt : current_points_) {
            if (pt.is_good && pt.num_observations > 3) {
                new_points.push_back(pt);
            }
        }
        
        current_points_ = new_points;
        
        keyframe_manager_->AddKeyframe(
            current_timestamp_,
            current_frame_,
            current_pose_,
            current_points_
        );
    }
    
    // Motion model güncelle
    UpdateMotionModel();
    
    return true;
}

void VisualOdometry::UpdateMotionModel() {
    if (frame_count_ > 1) {
        // Relative motion (translation + rotation)
        Sophus::SE3d delta = current_pose_ * last_pose_.inverse();
        
        // Smooth velocity update (damping factor 0.8)
        // Bu rotation için daha stabil olur
        if (frame_count_ == 2) {
            velocity_ = delta;
        } else {
            // Exponential smoothing
            Eigen::Vector3d trans = 0.8 * delta.translation() + 0.2 * velocity_.translation();
            Eigen::Vector3d rot = 0.8 * delta.so3().log() + 0.2 * velocity_.so3().log();
            velocity_ = Sophus::SE3d(Sophus::SO3d::exp(rot), trans);
        }
    }
}

Sophus::SE3d VisualOdometry::PredictPose() const {
    // Constant velocity model (translation + rotation)
    // Bu pitch/yaw/roll için çalışır
    return velocity_ * last_pose_;
}

void VisualOdometry::AddTrajectoryPoint() {
    PoseData pose_data;
    pose_data.timestamp = current_timestamp_;
    pose_data.pose = current_pose_.matrix();
    pose_data.is_keyframe = false;
    
    auto last_kf = keyframe_manager_->GetLastKeyframe();
    if (last_kf && last_kf->timestamp == current_timestamp_) {
        pose_data.is_keyframe = true;
    }
    
    trajectory_.push_back(pose_data);
}

void VisualOdometry::PrintStatistics() const {
    std::cout << "\n=== Visual Odometry Statistics ===" << std::endl;
    std::cout << "Total frames processed: " << frame_count_ << std::endl;
    std::cout << "Total keyframes: " << keyframe_manager_->GetNumKeyframes() << std::endl;
    std::cout << "Trajectory points: " << trajectory_.size() << std::endl;
    
    if (!trajectory_.empty()) {
        Eigen::Vector3d total_translation = current_pose_.translation();
        std::cout << "Total translation: " << total_translation.norm() << " meters" << std::endl;
        std::cout << "Final position: [" << total_translation.transpose() << "]" << std::endl;
    }
}
