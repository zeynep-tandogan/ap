#include "PointSelector.h"
#include <algorithm>

PointSelector::PointSelector(const CameraModel& cam) : cam_(cam) {}

std::vector<DirectPoint> PointSelector::SelectPoints(
    const cv::Mat& img,
    int target_num_points,
    int min_gradient_threshold) {
    
    std::vector<DirectPoint> candidates;
    
    // Gradient magnitude map oluştur
    cv::Mat img_float;
    if (img.channels() == 3) {
        cv::cvtColor(img, img_float, cv::COLOR_BGR2GRAY);
    } else {
        img.copyTo(img_float);
    }
    img_float.convertTo(img_float, CV_32F);
    
    // Sobel gradient
    cv::Mat grad_x, grad_y, grad_mag;
    cv::Sobel(img_float, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(img_float, grad_y, CV_32F, 0, 1, 3);
    cv::magnitude(grad_x, grad_y, grad_mag);
    
    // Threshold uygula ve nokta topla
    int border = 10;
    for (int y = border; y < img.rows - border; y++) {
        for (int x = border; x < img.cols - border; x++) {
            float mag = grad_mag.at<float>(y, x);
            
            if (mag > min_gradient_threshold) {
                DirectPoint pt;
                pt.u = x;
                pt.v = y;
                pt.intensity = img_float.at<float>(y, x);
                pt.gradient = Eigen::Vector2f(
                    grad_x.at<float>(y, x),
                    grad_y.at<float>(y, x)
                );
                pt.idepth = 0.5f;  // Initial guess: 2 meter
                pt.idepth_variance = 1.0f;
                
                candidates.push_back(pt);
            }
        }
    }
    
    // Gradient'e göre sırala ve en iyileri seç
    std::sort(candidates.begin(), candidates.end(),
        [](const DirectPoint& a, const DirectPoint& b) {
            return a.gradient.norm() > b.gradient.norm();
        });
    
    if (candidates.size() > target_num_points) {
        candidates.resize(target_num_points);
    }
    
    return candidates;
}

std::vector<DirectPoint> PointSelector::SelectPointsGrid(
    const cv::Mat& img,
    int grid_size,
    int points_per_cell) {
    
    std::vector<DirectPoint> points;
    
    cv::Mat img_float;
    if (img.channels() == 3) {
        cv::cvtColor(img, img_float, cv::COLOR_BGR2GRAY);
    } else {
        img.copyTo(img_float);
    }
    img_float.convertTo(img_float, CV_32F);
    
    // Gradient hesapla
    cv::Mat grad_x, grad_y, grad_mag;
    cv::Sobel(img_float, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(img_float, grad_y, CV_32F, 0, 1, 3);
    cv::magnitude(grad_x, grad_y, grad_mag);
    
    int cell_width = img.cols / grid_size;
    int cell_height = img.rows / grid_size;
    
    for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
            int x_start = j * cell_width;
            int y_start = i * cell_height;
            int x_end = std::min(x_start + cell_width, img.cols);
            int y_end = std::min(y_start + cell_height, img.rows);
            
            // Her cell'den en yüksek gradient'li noktaları seç
            std::vector<std::pair<float, cv::Point>> cell_candidates;
            
            for (int y = y_start + 5; y < y_end - 5; y++) {
                for (int x = x_start + 5; x < x_end - 5; x++) {
                    float mag = grad_mag.at<float>(y, x);
                    cell_candidates.push_back({mag, cv::Point(x, y)});
                }
            }
            
            std::sort(cell_candidates.begin(), cell_candidates.end(),
                [](const auto& a, const auto& b) { return a.first > b.first; });
            
            int num_to_select = std::min(points_per_cell, 
                                        (int)cell_candidates.size());
            
            for (int k = 0; k < num_to_select; k++) {
                // Gradient threshold düşürdük - texture-less için
                if (cell_candidates[k].first < 5) break;  // 10'dan 5'e
                
                cv::Point p = cell_candidates[k].second;
                DirectPoint pt;
                pt.u = p.x;
                pt.v = p.y;
                pt.intensity = img_float.at<float>(p.y, p.x);
                pt.gradient = Eigen::Vector2f(
                    grad_x.at<float>(p.y, p.x),
                    grad_y.at<float>(p.y, p.x)
                );
                pt.idepth = 0.5f;
                pt.idepth_variance = 1.0f;
                
                points.push_back(pt);
            }
        }
    }
    
    return points;
}

void PointSelector::ComputeGradients(
    const cv::Mat& img,
    std::vector<DirectPoint>& points) {
    
    cv::Mat img_float;
    if (img.channels() == 3) {
        cv::cvtColor(img, img_float, cv::COLOR_BGR2GRAY);
    } else {
        img.copyTo(img_float);
    }
    img_float.convertTo(img_float, CV_32F);
    
    for (auto& pt : points) {
        int x = static_cast<int>(pt.u);
        int y = static_cast<int>(pt.v);
        
        if (x > 0 && x < img.cols - 1 && y > 0 && y < img.rows - 1) {
            float dx = (img_float.at<float>(y, x + 1) - 
                       img_float.at<float>(y, x - 1)) * 0.5f;
            float dy = (img_float.at<float>(y + 1, x) - 
                       img_float.at<float>(y - 1, x)) * 0.5f;
            pt.gradient = Eigen::Vector2f(dx, dy);
        }
    }
}
