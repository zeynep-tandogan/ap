#include "include/VisualOdometry.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <map>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <filesystem>

namespace fs = std::filesystem;

// İki pose arasında interpolasyon (SLERP için)
Eigen::Matrix4d InterpolatePose(const Eigen::Matrix4d& pose1, 
                                const Eigen::Matrix4d& pose2, 
                                double t) {
    // t: 0 = pose1, 1 = pose2
    Sophus::SE3d se3_1(pose1);
    Sophus::SE3d se3_2(pose2);
    
    // Log-space interpolation
    Sophus::SE3d delta = se3_2 * se3_1.inverse();
    Sophus::SE3d::Tangent log_delta = delta.log();
    Sophus::SE3d interpolated = Sophus::SE3d::exp(t * log_delta) * se3_1;
    
    return interpolated.matrix();
}

// Başarısız frame'leri interpolasyon ile doldur
std::vector<PoseData> InterpolateMissingPoses(const std::vector<PoseData>& trajectory,
                                              int total_frames,
                                              double frame_rate = 30.0) {
    std::vector<PoseData> complete_trajectory;
    
    if (trajectory.empty()) {
        return complete_trajectory;
    }
    
    // Frame index'e göre sırala
    std::map<int, PoseData> pose_map;
    for (const auto& pose : trajectory) {
        int frame_idx = static_cast<int>(pose.timestamp * frame_rate + 0.5);
        pose_map[frame_idx] = pose;
    }
    
    // Tüm frame'ler için pose oluştur
    for (int i = 0; i < total_frames; i++) {
        if (pose_map.count(i)) {
            // Var olan pose
            complete_trajectory.push_back(pose_map[i]);
        } else {
            // Eksik pose - interpolasyon yap
            // En yakın önceki ve sonraki pose'ları bul
            int prev_idx = -1, next_idx = -1;
            
            for (int j = i - 1; j >= 0; j--) {
                if (pose_map.count(j)) {
                    prev_idx = j;
                    break;
                }
            }
            
            for (int j = i + 1; j < total_frames; j++) {
                if (pose_map.count(j)) {
                    next_idx = j;
                    break;
                }
            }
            
            PoseData interpolated;
            interpolated.timestamp = i / frame_rate;
            interpolated.is_keyframe = false;
            
            if (prev_idx >= 0 && next_idx >= 0) {
                // İki pose arasında interpolasyon
                double t = (double)(i - prev_idx) / (next_idx - prev_idx);
                interpolated.pose = InterpolatePose(pose_map[prev_idx].pose,
                                                   pose_map[next_idx].pose, t);
            } else if (prev_idx >= 0) {
                // Sadece önceki var, onu kullan
                interpolated.pose = pose_map[prev_idx].pose;
            } else if (next_idx >= 0) {
                // Sadece sonraki var, onu kullan
                interpolated.pose = pose_map[next_idx].pose;
            } else {
                // Hiçbiri yok, identity kullan
                interpolated.pose = Eigen::Matrix4d::Identity();
            }
            
            complete_trajectory.push_back(interpolated);
        }
    }
    
    return complete_trajectory;
}

// Trajectory'yi TUM formatında kaydet
void SaveTrajectoryTUM(const std::vector<PoseData>& trajectory, 
                       const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return;
    }
    
    file << std::fixed << std::setprecision(6);
    
    for (const auto& pose_data : trajectory) {
        Eigen::Matrix4d T = pose_data.pose;
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d t = T.block<3, 1>(0, 3);
        
        // Rotation matrix'i quaternion'a çevir
        Eigen::Quaterniond q(R);
        q.normalize();
        
        // Format: timestamp tx ty tz qx qy qz qw
        file << pose_data.timestamp << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    
    file.close();
    std::cout << "Trajectory saved to: " << filename << std::endl;
}

// Trajectory'yi KITTI formatında kaydet
void SaveTrajectoryKITTI(const std::vector<PoseData>& trajectory,
                         const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return;
    }
    
    file << std::fixed << std::setprecision(9);
    
    for (const auto& pose_data : trajectory) {
        Eigen::Matrix4d T = pose_data.pose;
        
        // 3x4 transformation matrix (KITTI format)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                file << T(i, j);
                if (i != 2 || j != 3) file << " ";
            }
        }
        file << "\n";
    }
    
    file.close();
    std::cout << "Trajectory saved to: " << filename << std::endl;
}

// Video dosyasından frame'leri işle
void ProcessVideo(const std::string& video_path,
                 const CalibrationParams& calib,
                 const std::string& output_dir) {
    
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open video: " << video_path << std::endl;
        return;
    }
    
    VisualOdometry vo(calib);
    vo.SetVerbose(false);  // Disable verbose for speed
    
    cv::Mat frame;
    int frame_id = 0;
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;  // Default FPS
    
    std::cout << "Processing video: " << video_path << std::endl;
    std::cout << "FPS: " << fps << std::endl;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (cap.read(frame)) {
        double timestamp = frame_id / fps;
        
        // Convert to grayscale for speed
        cv::Mat gray;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame;
        }
        
        bool success = vo.ProcessFrame(gray, timestamp);
        
        // Progress indicator every 50 frames
        if (frame_id % 50 == 0) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            float proc_fps = (elapsed > 0) ? frame_id / (float)elapsed : 0;
            std::cout << "\rFrame: " << frame_id << " - " << proc_fps << " fps" << std::flush;
        }
        
        if (!success && frame_id > 10) {
            std::cout << "\nWarning: Frame " << frame_id << " tracking failed!" << std::endl;
        }
        
        frame_id++;
    }
    std::cout << std::endl;
    
    cap.release();
    
    // İstatistikler
    vo.PrintStatistics();
    
    // RAW trajectory
    auto raw_trajectory = vo.GetTrajectory();
    
    // INTERPOLATED trajectory
    std::cout << "\nInterpolating missing poses..." << std::endl;
    auto complete_trajectory = InterpolateMissingPoses(raw_trajectory, 
                                                       frame_id, 
                                                       fps);
    
    std::cout << "Raw trajectory: " << raw_trajectory.size() << " poses" << std::endl;
    std::cout << "Complete trajectory: " << complete_trajectory.size() << " poses" << std::endl;
    
    // Trajectory'yi kaydet
    fs::create_directories(output_dir);
    SaveTrajectoryTUM(raw_trajectory, output_dir + "/trajectory_raw_tum.txt");
    SaveTrajectoryKITTI(raw_trajectory, output_dir + "/trajectory_raw_kitti.txt");
    SaveTrajectoryTUM(complete_trajectory, output_dir + "/trajectory_tum.txt");
    SaveTrajectoryKITTI(complete_trajectory, output_dir + "/trajectory_kitti.txt");
    
    std::cout << "\n✓ Saved trajectories to " << output_dir << std::endl;
}

// Image sequence'dan frame'leri işle
void ProcessImageSequence(const std::string& image_dir,
                         const CalibrationParams& calib,
                         const std::string& output_dir) {
    
    std::vector<std::string> image_files;
    for (const auto& entry : fs::directory_iterator(image_dir)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".webp") {
                image_files.push_back(entry.path().string());
            }
        }
    }
    
    std::sort(image_files.begin(), image_files.end());
    
    if (image_files.empty()) {
        std::cerr << "No images found in: " << image_dir << std::endl;
        return;
    }
    
    std::cout << "Found " << image_files.size() << " images" << std::endl;
    std::cout << "Processing (verbose output disabled for speed)..." << std::endl;
    
    VisualOdometry vo(calib);
    vo.SetVerbose(false);  // Disable verbose for speed
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (size_t i = 0; i < image_files.size(); i++) {
        cv::Mat frame = cv::imread(image_files[i], cv::IMREAD_GRAYSCALE);  // Load as grayscale for speed
        if (frame.empty()) {
            std::cerr << "Cannot read image: " << image_files[i] << std::endl;
            continue;
        }
        
        double timestamp = i * 0.033;  // Assume 30 FPS
        
        bool success = vo.ProcessFrame(frame, timestamp);
        
        // Progress indicator every 50 frames
        if (i % 50 == 0) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            float progress = 100.0f * i / image_files.size();
            float fps = (elapsed > 0) ? i / (float)elapsed : 0;
            std::cout << "\rProgress: " << std::fixed << std::setprecision(1) 
                     << progress << "% (" << i << "/" << image_files.size() 
                     << ") - " << fps << " fps" << std::flush;
        }
        
        if (!success && i > 10) {  // Only report failures after initialization
            std::cout << "\nWarning: Frame " << i << " tracking failed!" << std::endl;
        }
    }
    std::cout << std::endl;
    
    // İstatistikler
    vo.PrintStatistics();
    
    // RAW trajectory (sadece başarılı frame'ler)
    auto raw_trajectory = vo.GetTrajectory();
    
    // INTERPOLATED trajectory (tüm frame'ler için)
    std::cout << "\nInterpolating missing poses..." << std::endl;
    auto complete_trajectory = InterpolateMissingPoses(raw_trajectory, 
                                                       image_files.size(), 
                                                       30.0);
    
    std::cout << "Raw trajectory: " << raw_trajectory.size() << " poses" << std::endl;
    std::cout << "Complete trajectory: " << complete_trajectory.size() << " poses" << std::endl;
    
    // Trajectory'leri kaydet
    fs::create_directories(output_dir);
    
    // Raw (sadece başarılı olanlar)
    SaveTrajectoryTUM(raw_trajectory, output_dir + "/trajectory_raw_tum.txt");
    SaveTrajectoryKITTI(raw_trajectory, output_dir + "/trajectory_raw_kitti.txt");
    
    // Complete (interpolasyonlu)
    SaveTrajectoryTUM(complete_trajectory, output_dir + "/trajectory_tum.txt");
    SaveTrajectoryKITTI(complete_trajectory, output_dir + "/trajectory_kitti.txt");
    
    std::cout << "\n✓ Saved trajectories:" << std::endl;
    std::cout << "  - " << output_dir << "/trajectory_tum.txt (interpolated)" << std::endl;
    std::cout << "  - " << output_dir << "/trajectory_raw_tum.txt (raw)" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "=== Direct Visual Odometry ===" << std::endl;
    std::cout << "Prensesim için hazırlandı 👑\n" << std::endl;
    
    // Kalibrasyon parametreleri - THYZ Dataset için
    // FocalLength: [1413.3, 1418.8]
    // PrincipalPoint: [950.0639, 543.3796]
    // ImageSize: [1080, 1920] -> height x width
    CalibrationParams calib;
    calib.fx = 1413.3;      // Focal length X
    calib.fy = 1418.8;      // Focal length Y
    calib.cx = 950.0639;    // Principal point X
    calib.cy = 543.3796;    // Principal point Y
    calib.width = 1920;     // Image width
    calib.height = 1080;    // Image height
    
    if (argc < 2) {
        std::cout << "Usage:" << std::endl;
        std::cout << "  " << argv[0] << " <video_file> [output_dir]" << std::endl;
        std::cout << "  " << argv[0] << " <image_directory> [output_dir]" << std::endl;
        std::cout << "\nExample:" << std::endl;
        std::cout << "  " << argv[0] << " video.mp4 results/" << std::endl;
        std::cout << "  " << argv[0] << " ../thyz_1/THYZ_2025_Oturum_1/ results/" << std::endl;
        return 1;
    }
    
    std::string input_path = argv[1];
    std::string output_dir = (argc > 2) ? argv[2] : "results";
    
    // Input'un video mu yoksa dizin mi olduğunu kontrol et
    if (fs::is_directory(input_path)) {
        ProcessImageSequence(input_path, calib, output_dir);
    } else {
        ProcessVideo(input_path, calib, output_dir);
    }
    
    std::cout << "\nDone! 🎉" << std::endl;
    
    return 0;
}
