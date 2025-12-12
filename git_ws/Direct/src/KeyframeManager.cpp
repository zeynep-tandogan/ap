#include "KeyframeManager.h"
#include <iostream>

KeyframeManager::KeyframeManager()
    : next_keyframe_id_(0),
      min_translation_threshold_(0.15),     // 15 cm (biraz arttırdık)
      min_rotation_threshold_(0.15),        // ~8.6 derece (rotation için hassas)
      min_tracked_points_ratio_(70) {}      // %70 (daha toleranslı)

void KeyframeManager::AddKeyframe(
    double timestamp,
    const cv::Mat& image,
    const Sophus::SE3d& pose,
    const std::vector<DirectPoint>& points) {
    
    auto kf = std::make_shared<Keyframe>(
        next_keyframe_id_++,
        timestamp,
        image,
        pose
    );
    kf->points = points;
    
    keyframes_.push_back(kf);
    
    // std::cout << "Added keyframe #" << kf->id 
    //           << " with " << points.size() << " points" << std::endl;
}

bool KeyframeManager::NeedNewKeyframe(
    const Sophus::SE3d& current_pose,
    int num_tracked_points) const {
    
    if (keyframes_.empty()) {
        return true;  // İlk keyframe
    }
    
    auto last_kf = keyframes_.back();
    
    // Relative pose
    Sophus::SE3d relative_pose = current_pose * last_kf->pose.inverse();
    
    // Translation ve rotation hesapla
    double translation = relative_pose.translation().norm();
    double rotation = relative_pose.so3().log().norm();
    
    // Takip edilen nokta sayısı kontrolü
    int initial_points = last_kf->points.size();
    double tracked_ratio = 100.0 * num_tracked_points / (initial_points + 1);
    
    bool need_kf = false;
    
    if (translation > min_translation_threshold_) {
        // Keyframe needed: translation
        need_kf = true;
    }
    
    if (rotation > min_rotation_threshold_) {
        // Keyframe needed: rotation (pitch/yaw/roll için önemli)
        need_kf = true;
    }
    
    if (tracked_ratio < min_tracked_points_ratio_) {
        // Keyframe needed: too few points tracked
        need_kf = true;
    }
    
    return need_kf;
}

std::shared_ptr<Keyframe> KeyframeManager::GetLastKeyframe() const {
    if (keyframes_.empty()) {
        return nullptr;
    }
    return keyframes_.back();
}

std::shared_ptr<Keyframe> KeyframeManager::GetKeyframe(int id) const {
    for (const auto& kf : keyframes_) {
        if (kf->id == id) {
            return kf;
        }
    }
    return nullptr;
}
