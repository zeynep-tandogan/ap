#ifndef POINT_SELECTOR_H
#define POINT_SELECTOR_H

#include "DirectPoint.h"
#include "CameraModel.h"
#include <opencv2/opencv.hpp>
#include <vector>

class PointSelector {
public:
    PointSelector(const CameraModel& cam);
    
    // Frame'den nokta seç (gradient-based)
    std::vector<DirectPoint> SelectPoints(
        const cv::Mat& img,
        int target_num_points = 2000,
        int min_gradient_threshold = 10
    );
    
    // Gradient hesapla
    void ComputeGradients(
        const cv::Mat& img,
        std::vector<DirectPoint>& points
    );
    
    // Grid-based selection (uniform distribution)
    std::vector<DirectPoint> SelectPointsGrid(
        const cv::Mat& img,
        int grid_size = 20,
        int points_per_cell = 5
    );
    
private:
    CameraModel cam_;
    
    // Gradient magnitude hesapla
    float ComputeGradientMagnitude(const cv::Mat& img, int x, int y);
    
    // Bilinear interpolation ile gradient
    Eigen::Vector2f ComputeGradient(const cv::Mat& img, float x, float y);
};

#endif // POINT_SELECTOR_H
