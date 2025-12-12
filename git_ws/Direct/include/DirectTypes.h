#ifndef DIRECT_TYPES_H
#define DIRECT_TYPES_H

#include <Eigen/Core>
#include <vector>

// Temel tipler
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// Pose bilgisi
struct PoseData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix4d pose;  // 4x4 transformation matrix
    bool is_keyframe;
    
    PoseData() : timestamp(0.0), is_keyframe(false) {
        pose = Eigen::Matrix4d::Identity();
    }
};

// Kalibrasyon parametreleri
struct CalibrationParams {
    float fx, fy, cx, cy;
    int width, height;
    
    CalibrationParams() : fx(0), fy(0), cx(0), cy(0), width(0), height(0) {}
    
    CalibrationParams(float fx_, float fy_, float cx_, float cy_, int w, int h)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_), width(w), height(h) {}
};

#endif // DIRECT_TYPES_H
