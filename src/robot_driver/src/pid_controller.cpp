#include "robot_driver/pid_controller.hpp"
#include <algorithm>
#include <cmath>

PIDController::PIDController(double kp, double ki, double kd,
                             double max_output, double max_integral)
    : kp_(kp), ki_(ki), kd_(kd),
      max_output_(max_output), max_integral_(max_integral) {}

double PIDController::compute(double setpoint, double measured, double dt) {
    if (dt <= 0.0) {
        return 0.0;
    }

    double error = setpoint - measured;

    // Proportional
    double p_term = kp_ * error;

    // Integral with anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);
    double i_term = ki_ * integral_;

    // Derivative (on error, skip first iteration)
    double d_term = 0.0;
    if (!first_run_) {
        d_term = kd_ * (error - prev_error_) / dt;
    }
    first_run_ = false;
    prev_error_ = error;

    // Sum and clamp
    double output = p_term + i_term + d_term;
    return std::clamp(output, -max_output_, max_output_);
}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_run_ = true;
}

void PIDController::set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::set_limits(double max_output, double max_integral) {
    max_output_ = max_output;
    max_integral_ = max_integral;
}
