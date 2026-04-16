#include "robot_odom/odometry.hpp"
#include <cmath>

DiffDriveOdometry::DiffDriveOdometry(double track_width)
    : track_width_(track_width) {}

void DiffDriveOdometry::update(double left_vel, double right_vel, double dt) {
    if (dt <= 0.0) return;

    linear_vel_  = (right_vel + left_vel) / 2.0;
    angular_vel_ = (right_vel - left_vel) / track_width_;

    // Integrate using mid-point method for better accuracy
    double delta_theta = angular_vel_ * dt;
    double mid_theta   = theta_ + delta_theta / 2.0;

    x_     += linear_vel_ * std::cos(mid_theta) * dt;
    y_     += linear_vel_ * std::sin(mid_theta) * dt;
    theta_ += delta_theta;

    // Normalize theta to [-pi, pi]
    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
}

void DiffDriveOdometry::reset() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    linear_vel_ = 0.0;
    angular_vel_ = 0.0;
}
