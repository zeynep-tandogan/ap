#ifndef DEPTH_FILTER_H
#define DEPTH_FILTER_H

#include "DirectPoint.h"
#include "CameraModel.h"
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class DepthFilter {
public:
    DepthFilter(const CameraModel& cam);
    
    // Depth güncelleme (triangulation-based)
    void UpdateDepths(
        std::vector<DirectPoint>& points,
        const cv::Mat& ref_img,
        const cv::Mat& curr_img,
        const Sophus::SE3d& ref_pose,
        const Sophus::SE3d& curr_pose
    );
    
    // İlk depth tahmini (sabit değer)
    void InitializeDepths(
        std::vector<DirectPoint>& points,
        float initial_depth = 2.0f
    );
    
private:
    CameraModel cam_;
    
    // Epipolar search
    bool SearchEpipolarLine(
        const cv::Mat& ref_img,
        const cv::Mat& curr_img,
        const DirectPoint& pt,
        const Sophus::SE3d& T_curr_ref,
        float& best_depth,
        float& depth_variance
    );
    
    double GetPixelValue(const cv::Mat& img, float x, float y) const;
};

#endif // DEPTH_FILTER_H
