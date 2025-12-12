#include "DirectTracker.h"
#include <iostream>

DirectTracker::DirectTracker(const CameraModel& cam)
    : cam_(cam),
      pyramid_levels_(3),
      max_iterations_(10),
      convergence_threshold_(1e-4),
      huber_threshold_(9.0),
      last_error_(0),
      num_inliers_(0) {}

void DirectTracker::BuildPyramid(const cv::Mat& img, std::vector<cv::Mat>& pyramid) {
    pyramid.clear();
    pyramid.resize(pyramid_levels_);
    
    cv::Mat img_float;
    if (img.channels() == 3) {
        cv::cvtColor(img, img_float, cv::COLOR_BGR2GRAY);
    } else {
        img.copyTo(img_float);
    }
    img_float.convertTo(pyramid[0], CV_32F);
    
    for (int i = 1; i < pyramid_levels_; i++) {
        cv::pyrDown(pyramid[i - 1], pyramid[i]);
    }
}

double DirectTracker::GetPixelValue(const cv::Mat& img, float x, float y) const {
    if (x < 0 || x >= img.cols - 1 || y < 0 || y >= img.rows - 1) {
        return 0;
    }
    
    int x0 = static_cast<int>(x);
    int y0 = static_cast<int>(y);
    float dx = x - x0;
    float dy = y - y0;
    
    float v00 = img.at<float>(y0, x0);
    float v01 = img.at<float>(y0, x0 + 1);
    float v10 = img.at<float>(y0 + 1, x0);
    float v11 = img.at<float>(y0 + 1, x0 + 1);
    
    return (1 - dx) * (1 - dy) * v00 +
           dx * (1 - dy) * v01 +
           (1 - dx) * dy * v10 +
           dx * dy * v11;
}

void DirectTracker::ComputeJacobian(
    const DirectPoint& pt,
    const Sophus::SE3d& pose,
    const Eigen::Vector2d& projected,
    const cv::Mat& curr_img,
    Eigen::Matrix<double, 1, 6>& J) {
    
    // 3D nokta (referans frame'de)
    Eigen::Vector3d p3d = cam_.backProject(static_cast<double>(pt.u), static_cast<double>(pt.v), 1.0 / static_cast<double>(pt.idepth));
    
    // Transform edilmiş nokta (current frame'de)
    Eigen::Vector3d p_curr = pose * p3d;
    double X = p_curr[0], Y = p_curr[1], Z = p_curr[2];
    double Z_inv = 1.0 / Z;
    double Z_inv2 = Z_inv * Z_inv;
    
    // Image gradient
    float dx = (GetPixelValue(curr_img, projected[0] + 1, projected[1]) -
                GetPixelValue(curr_img, projected[0] - 1, projected[1])) * 0.5f;
    float dy = (GetPixelValue(curr_img, projected[0], projected[1] + 1) -
                GetPixelValue(curr_img, projected[0], projected[1] - 1)) * 0.5f;
    
    // Projeksiyon Jacobian'ı: ∂u/∂p
    Eigen::Matrix<double, 2, 3> proj_jac;
    proj_jac << cam_.fx() * Z_inv,  0,  -cam_.fx() * X * Z_inv2,
                0,  cam_.fy() * Z_inv,  -cam_.fy() * Y * Z_inv2;
    
    // SE3 Jacobian'ı: ∂p/∂ξ
    Eigen::Matrix<double, 3, 6> se3_jac;
    se3_jac << 0, Z, -Y, 1, 0, 0,
              -Z, 0,  X, 0, 1, 0,
               Y,-X,  0, 0, 0, 1;
    
    // Chain rule
    Eigen::Matrix<double, 1, 2> grad;
    grad << dx, dy;
    
    J = grad * proj_jac * se3_jac;
}

double DirectTracker::HuberWeight(double residual) const {
    double abs_res = std::abs(residual);
    if (abs_res < huber_threshold_) {
        return 1.0;
    } else {
        return huber_threshold_ / abs_res;
    }
}

bool DirectTracker::Track(
    const cv::Mat& ref_img,
    const cv::Mat& curr_img,
    std::vector<DirectPoint>& points,
    Sophus::SE3d& pose,
    const Sophus::SE3d& initial_pose) {
    
    pose = initial_pose;
    
    // Piramit oluştur
    BuildPyramid(ref_img, ref_pyramid_);
    BuildPyramid(curr_img, curr_pyramid_);
    
    // Coarse-to-fine tracking
    for (int lvl = pyramid_levels_ - 1; lvl >= 0; lvl--) {
        float scale = 1.0f / (1 << lvl);
        
        for (int iter = 0; iter < max_iterations_; iter++) {
            Matrix6d H = Matrix6d::Zero();
            Vector6d b = Vector6d::Zero();
            double total_cost = 0;
            int valid_points = 0;
            std::vector<double> residuals;
            
            for (auto& pt : points) {
                if (!pt.is_good) continue;
                
                // 3D nokta
                Eigen::Vector3d p3d = cam_.backProject(static_cast<double>(pt.u), static_cast<double>(pt.v), 1.0 / static_cast<double>(pt.idepth));
                
                // Current frame'e projeksiyon
                Eigen::Vector3d p_curr = pose * p3d;
                if (p_curr[2] <= 0.1) continue;
                
                Eigen::Vector2d uv_curr = cam_.project(p_curr);
                uv_curr *= scale;
                
                if (!cam_.isInFrame(uv_curr.cast<float>(), 5)) continue;
                
                // Photometric residual
                float I_ref = pt.intensity;
                float I_curr = GetPixelValue(curr_pyramid_[lvl], 
                                            uv_curr[0], uv_curr[1]);
                double residual = I_curr - I_ref;
                residuals.push_back(residual);
                
                // Jacobian
                Eigen::Matrix<double, 1, 6> J;
                ComputeJacobian(pt, pose, uv_curr, curr_pyramid_[lvl], J);
                
                // Robust weight
                double weight = HuberWeight(residual);
                
                // Accumulate
                H += weight * J.transpose() * J;
                b += weight * residual * J.transpose();
                
                total_cost += residual * residual;
                valid_points++;
            }
            
            // Minimum nokta kontrolü - daha toleranslı
            if (valid_points < 30) {  // 50'den 30'a düşürdük
                // Çok az nokta, ama yine de devam et
                if (lvl == 0 && valid_points < 20) {  // En ince level'da 20'den az ise fail
                    // std::cout << "Too few valid points: " << valid_points << std::endl;
                    return false;
                }
                // Üst level'larda devam et
            }
            
            // Solve: H * delta = -b
            Vector6d delta = H.ldlt().solve(-b);
            
            // Update pose
            pose = Sophus::SE3d::exp(delta) * pose;
            
            last_error_ = total_cost / valid_points;
            num_inliers_ = valid_points;
            
            // Convergence check
            if (delta.norm() < convergence_threshold_) {
                break;
            }
        }
    }
    
    return true;
}

void DirectTracker::FilterOutliers(
    std::vector<DirectPoint>& points,
    const std::vector<double>& residuals) {
    
    if (residuals.empty()) return;
    
    // Median hesapla
    std::vector<double> sorted_res = residuals;
    std::sort(sorted_res.begin(), sorted_res.end());
    double median = sorted_res[sorted_res.size() / 2];
    
    // MAD (Median Absolute Deviation)
    std::vector<double> abs_dev;
    for (double r : residuals) {
        abs_dev.push_back(std::abs(r - median));
    }
    std::sort(abs_dev.begin(), abs_dev.end());
    double mad = abs_dev[abs_dev.size() / 2];
    
    double threshold = median + 3.0 * mad;
    
    // Outlier'ları işaretle
    for (size_t i = 0; i < points.size() && i < residuals.size(); i++) {
        if (std::abs(residuals[i]) > threshold) {
            points[i].is_good = false;
        }
    }
}
