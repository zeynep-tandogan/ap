#ifndef DIRECT_TRACKER_H
#define DIRECT_TRACKER_H

#include "DirectTypes.h"
#include "DirectPoint.h"
#include "CameraModel.h"
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <vector>

class DirectTracker {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    DirectTracker(const CameraModel& cam);
    
    // Ana tracking fonksiyonu
    bool Track(
        const cv::Mat& ref_img,
        const cv::Mat& curr_img,
        std::vector<DirectPoint>& points,
        Sophus::SE3d& pose,
        const Sophus::SE3d& initial_pose = Sophus::SE3d()
    );
    
    // Parametreler
    void SetPyramidLevels(int levels) { pyramid_levels_ = levels; }
    void SetMaxIterations(int iters) { max_iterations_ = iters; }
    void SetHuberThreshold(double threshold) { huber_threshold_ = threshold; }
    
    // İstatistikler
    double GetLastError() const { return last_error_; }
    int GetNumInliers() const { return num_inliers_; }
    
private:
    CameraModel cam_;
    
    // Parametreler
    int pyramid_levels_;
    int max_iterations_;
    double convergence_threshold_;
    double huber_threshold_;
    
    // İstatistikler
    double last_error_;
    int num_inliers_;
    
    // Piramit
    std::vector<cv::Mat> ref_pyramid_;
    std::vector<cv::Mat> curr_pyramid_;
    
    // Yardımcı fonksiyonlar
    void BuildPyramid(const cv::Mat& img, std::vector<cv::Mat>& pyramid);
    
    double GetPixelValue(const cv::Mat& img, float x, float y) const;
    
    void ComputeJacobian(
        const DirectPoint& pt,
        const Sophus::SE3d& pose,
        const Eigen::Vector2d& projected,
        const cv::Mat& curr_img,
        Eigen::Matrix<double, 1, 6>& J
    );
    
    double HuberWeight(double residual) const;
    
    void FilterOutliers(
        std::vector<DirectPoint>& points,
        const std::vector<double>& residuals
    );
};

#endif // DIRECT_TRACKER_H
