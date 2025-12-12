#include "DepthFilter.h"
#include <iostream>

DepthFilter::DepthFilter(const CameraModel& cam) : cam_(cam) {}

void DepthFilter::InitializeDepths(
    std::vector<DirectPoint>& points,
    float initial_depth) {
    
    for (auto& pt : points) {
        pt.idepth = 1.0f / initial_depth;
        pt.idepth_variance = 1.0f;
    }
}

double DepthFilter::GetPixelValue(const cv::Mat& img, float x, float y) const {
    if (x < 0 || x >= img.cols - 1 || y < 0 || y >= img.rows - 1) {
        return 0;
    }
    
    int x0 = static_cast<int>(x);
    int y0 = static_cast<int>(y);
    float dx = x - x0;
    float dy = y - y0;
    
    cv::Mat img_float;
    if (img.type() != CV_32F) {
        img.convertTo(img_float, CV_32F);
    } else {
        img_float = img;
    }
    
    float v00 = img_float.at<float>(y0, x0);
    float v01 = img_float.at<float>(y0, x0 + 1);
    float v10 = img_float.at<float>(y0 + 1, x0);
    float v11 = img_float.at<float>(y0 + 1, x0 + 1);
    
    return (1 - dx) * (1 - dy) * v00 +
           dx * (1 - dy) * v01 +
           (1 - dx) * dy * v10 +
           dx * dy * v11;
}

bool DepthFilter::SearchEpipolarLine(
    const cv::Mat& ref_img,
    const cv::Mat& curr_img,
    const DirectPoint& pt,
    const Sophus::SE3d& T_curr_ref,
    float& best_depth,
    float& depth_variance) {
    
    // Depth aralığı
    float min_depth = 0.5f;
    float max_depth = 10.0f;
    int num_samples = 50;
    
    float best_ncc = -1.0f;
    best_depth = 1.0f / pt.idepth;
    
    // Referans patch
    int patch_size = 5;
    int half_patch = patch_size / 2;
    
    std::vector<float> ref_patch;
    for (int dy = -half_patch; dy <= half_patch; dy++) {
        for (int dx = -half_patch; dx <= half_patch; dx++) {
            float val = GetPixelValue(ref_img, pt.u + dx, pt.v + dy);
            ref_patch.push_back(val);
        }
    }
    
    // Epipolar line boyunca ara
    for (int i = 0; i < num_samples; i++) {
        float depth = min_depth + (max_depth - min_depth) * i / num_samples;
        
        // 3D nokta
        Eigen::Vector3d p3d_ref = cam_.backProject(static_cast<double>(pt.u), static_cast<double>(pt.v), static_cast<double>(depth));
        
        // Current frame'e transform
        Eigen::Vector3d p3d_curr = T_curr_ref * p3d_ref;
        
        if (p3d_curr[2] <= 0) continue;
        
        // Projeksiyon
        Eigen::Vector2d uv_curr = cam_.project(p3d_curr);
        
        if (!cam_.isInFrame(uv_curr.cast<float>(), patch_size)) continue;
        
        // Current patch
        std::vector<float> curr_patch;
        for (int dy = -half_patch; dy <= half_patch; dy++) {
            for (int dx = -half_patch; dx <= half_patch; dx++) {
                float val = GetPixelValue(curr_img, uv_curr[0] + dx, uv_curr[1] + dy);
                curr_patch.push_back(val);
            }
        }
        
        // NCC hesapla
        float mean_ref = 0, mean_curr = 0;
        for (size_t j = 0; j < ref_patch.size(); j++) {
            mean_ref += ref_patch[j];
            mean_curr += curr_patch[j];
        }
        mean_ref /= ref_patch.size();
        mean_curr /= curr_patch.size();
        
        float numerator = 0, denom_ref = 0, denom_curr = 0;
        for (size_t j = 0; j < ref_patch.size(); j++) {
            float diff_ref = ref_patch[j] - mean_ref;
            float diff_curr = curr_patch[j] - mean_curr;
            numerator += diff_ref * diff_curr;
            denom_ref += diff_ref * diff_ref;
            denom_curr += diff_curr * diff_curr;
        }
        
        float ncc = numerator / (std::sqrt(denom_ref * denom_curr) + 1e-10f);
        
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_depth = depth;
        }
    }
    
    depth_variance = 0.1f;  // Basit bir tahmin
    
    return best_ncc > 0.7f;
}

void DepthFilter::UpdateDepths(
    std::vector<DirectPoint>& points,
    const cv::Mat& ref_img,
    const cv::Mat& curr_img,
    const Sophus::SE3d& ref_pose,
    const Sophus::SE3d& curr_pose) {
    
    Sophus::SE3d T_curr_ref = curr_pose * ref_pose.inverse();
    
    // Baseline kontrolü
    double baseline = T_curr_ref.translation().norm();
    if (baseline < 0.05) {
        return;  // Çok az hareket, depth güncellemesi yapma
    }
    
    for (auto& pt : points) {
        if (!pt.is_good) continue;
        
        float new_depth, variance;
        if (SearchEpipolarLine(ref_img, curr_img, pt, T_curr_ref, 
                              new_depth, variance)) {
            // Bayesian update
            float old_idepth = pt.idepth;
            float old_var = pt.idepth_variance;
            float new_idepth = 1.0f / new_depth;
            
            float updated_var = (old_var * variance) / (old_var + variance);
            float updated_idepth = updated_var * (old_idepth / old_var + new_idepth / variance);
            
            pt.idepth = updated_idepth;
            pt.idepth_variance = updated_var;
            pt.num_observations++;
        }
    }
}
