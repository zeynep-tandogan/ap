#include "CameraModel.h"

CameraModel::CameraModel() 
    : fx_(0), fy_(0), cx_(0), cy_(0), width_(0), height_(0) {}

CameraModel::CameraModel(const CalibrationParams& params)
    : fx_(params.fx), fy_(params.fy), 
      cx_(params.cx), cy_(params.cy),
      width_(params.width), height_(params.height) {}

Eigen::Vector2f CameraModel::project(const Eigen::Vector3f& p3d) const {
    return Eigen::Vector2f(
        fx_ * p3d[0] / p3d[2] + cx_,
        fy_ * p3d[1] / p3d[2] + cy_
    );
}

Eigen::Vector2d CameraModel::project(const Eigen::Vector3d& p3d) const {
    return Eigen::Vector2d(
        fx_ * p3d[0] / p3d[2] + cx_,
        fy_ * p3d[1] / p3d[2] + cy_
    );
}

Eigen::Vector3f CameraModel::backProject(float u, float v, float depth) const {
    return Eigen::Vector3f(
        (u - cx_) * depth / fx_,
        (v - cy_) * depth / fy_,
        depth
    );
}

Eigen::Vector3d CameraModel::backProject(double u, double v, double depth) const {
    return Eigen::Vector3d(
        (u - cx_) * depth / fx_,
        (v - cy_) * depth / fy_,
        depth
    );
}

bool CameraModel::isInFrame(const Eigen::Vector2f& uv, int boundary) const {
    return (uv[0] >= boundary && uv[0] < width_ - boundary &&
            uv[1] >= boundary && uv[1] < height_ - boundary);
}

bool CameraModel::isInFrame(float u, float v, int boundary) const {
    return (u >= boundary && u < width_ - boundary &&
            v >= boundary && v < height_ - boundary);
}
