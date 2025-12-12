#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include "DirectTypes.h"
#include <Eigen/Core>

class CameraModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    CameraModel();
    CameraModel(const CalibrationParams& params);
    
    // Projeksiyon: 3D -> 2D
    Eigen::Vector2f project(const Eigen::Vector3f& p3d) const;
    Eigen::Vector2d project(const Eigen::Vector3d& p3d) const;
    
    // Back-projeksiyon: 2D + depth -> 3D
    Eigen::Vector3f backProject(float u, float v, float depth) const;
    Eigen::Vector3d backProject(double u, double v, double depth) const;
    
    // Frame sınırları kontrolü
    bool isInFrame(const Eigen::Vector2f& uv, int boundary = 2) const;
    bool isInFrame(float u, float v, int boundary = 2) const;
    
    // Getter'lar
    float fx() const { return fx_; }
    float fy() const { return fy_; }
    float cx() const { return cx_; }
    float cy() const { return cy_; }
    int width() const { return width_; }
    int height() const { return height_; }
    
private:
    float fx_, fy_, cx_, cy_;
    int width_, height_;
};

#endif // CAMERA_MODEL_H
