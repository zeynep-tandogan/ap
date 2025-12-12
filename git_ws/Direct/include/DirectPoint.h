#ifndef DIRECT_POINT_H
#define DIRECT_POINT_H

#include <Eigen/Core>

struct DirectPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Piksel koordinatları (referans frame'de)
    float u, v;
    
    // Inverse depth ve belirsizliği
    float idepth;
    float idepth_variance;
    
    // Photometric bilgi
    float intensity;
    Eigen::Vector2f gradient;  // Image gradient (dx, dy)
    
    // Durum bilgisi
    bool is_good;
    int age;                   // Kaç frame'dir takip ediliyor
    float last_residual;       // Son photometric error
    int num_observations;      // Kaç kez gözlemlendi
    
    DirectPoint() 
        : u(0), v(0), idepth(0), idepth_variance(1.0f),
          intensity(0), is_good(true), age(0), 
          last_residual(0), num_observations(0) {
        gradient.setZero();
    }
    
    DirectPoint(float u_, float v_, float intensity_, float idepth_init = 0.5f)
        : u(u_), v(v_), intensity(intensity_), idepth(idepth_init),
          idepth_variance(1.0f), is_good(true), age(0),
          last_residual(0), num_observations(1) {
        gradient.setZero();
    }
};

#endif // DIRECT_POINT_H
