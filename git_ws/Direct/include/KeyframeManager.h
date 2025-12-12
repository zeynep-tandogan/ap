#ifndef KEYFRAME_MANAGER_H
#define KEYFRAME_MANAGER_H

#include "DirectTypes.h"
#include "DirectPoint.h"
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <vector>
#include <memory>

struct Keyframe {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    int id;
    double timestamp;
    cv::Mat image;
    Sophus::SE3d pose;
    std::vector<DirectPoint> points;
    
    Keyframe(int id_, double ts, const cv::Mat& img, const Sophus::SE3d& pose_)
        : id(id_), timestamp(ts), image(img.clone()), pose(pose_) {}
};

class KeyframeManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    KeyframeManager();
    
    // Yeni keyframe ekle
    void AddKeyframe(
        double timestamp,
        const cv::Mat& image,
        const Sophus::SE3d& pose,
        const std::vector<DirectPoint>& points
    );
    
    // Keyframe gerekli mi?
    bool NeedNewKeyframe(
        const Sophus::SE3d& current_pose,
        int num_tracked_points
    ) const;
    
    // Getter'lar
    std::shared_ptr<Keyframe> GetLastKeyframe() const;
    std::shared_ptr<Keyframe> GetKeyframe(int id) const;
    int GetNumKeyframes() const { return keyframes_.size(); }
    
    // Tüm keyframe'leri al
    const std::vector<std::shared_ptr<Keyframe>>& GetAllKeyframes() const {
        return keyframes_;
    }
    
private:
    std::vector<std::shared_ptr<Keyframe>> keyframes_;
    int next_keyframe_id_;
    
    // Parametreler
    double min_translation_threshold_;  // Minimum hareket (metre)
    double min_rotation_threshold_;     // Minimum rotasyon (radyan)
    int min_tracked_points_ratio_;      // Minimum takip edilen nokta oranı
};

#endif // KEYFRAME_MANAGER_H
