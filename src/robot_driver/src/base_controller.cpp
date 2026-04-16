#include "robot_driver/base_controller.hpp"
#include <algorithm>
#include <cmath>

BaseController::BaseController(const Config& cfg)
    : cfg_(cfg) {
    // Create four motors
    motor_fl_ = std::make_unique<Motor>(
        cfg_.gpio_chip, cfg_.fl.in1, cfg_.fl.in2, cfg_.fl.ena, cfg_.pwm_frequency);
    motor_fr_ = std::make_unique<Motor>(
        cfg_.gpio_chip, cfg_.fr.in1, cfg_.fr.in2, cfg_.fr.ena, cfg_.pwm_frequency);
    motor_rl_ = std::make_unique<Motor>(
        cfg_.gpio_chip, cfg_.rl.in1, cfg_.rl.in2, cfg_.rl.ena, cfg_.pwm_frequency);
    motor_rr_ = std::make_unique<Motor>(
        cfg_.gpio_chip, cfg_.rr.in1, cfg_.rr.in2, cfg_.rr.ena, cfg_.pwm_frequency);
}

BaseController::~BaseController() {
    stop_all();
}

void BaseController::set_target(double linear_x, double angular_z) {
    // Clamp inputs
    linear_x  = std::clamp(linear_x, -cfg_.max_linear_speed, cfg_.max_linear_speed);
    angular_z = std::clamp(angular_z, -cfg_.max_angular_speed, cfg_.max_angular_speed);

    // Differential drive inverse kinematics
    target_left_vel_  = linear_x - angular_z * cfg_.track_width / 2.0;
    target_right_vel_ = linear_x + angular_z * cfg_.track_width / 2.0;

    // Scale down if either side exceeds max speed
    double max_vel = std::max(std::abs(target_left_vel_), std::abs(target_right_vel_));
    if (max_vel > cfg_.max_linear_speed) {
        double scale = cfg_.max_linear_speed / max_vel;
        target_left_vel_  *= scale;
        target_right_vel_ *= scale;
    }
}

void BaseController::update(double measured_left, double measured_right, double dt) {
    if (cfg_.open_loop) {
        // Open-loop: map target velocity directly to duty cycle
        output_left_duty_  = target_left_vel_ / cfg_.max_linear_speed * cfg_.max_duty_cycle;
        output_right_duty_ = target_right_vel_ / cfg_.max_linear_speed * cfg_.max_duty_cycle;
    } else {
        // Closed-loop PID
        output_left_duty_  = pid_left_.compute(target_left_vel_, measured_left, dt);
        output_right_duty_ = pid_right_.compute(target_right_vel_, measured_right, dt);
    }

    // Clamp duty cycle
    output_left_duty_  = std::clamp(output_left_duty_, -cfg_.max_duty_cycle, cfg_.max_duty_cycle);
    output_right_duty_ = std::clamp(output_right_duty_, -cfg_.max_duty_cycle, cfg_.max_duty_cycle);

    // Apply to motors (same-side wheels get the same command)
    if (motor_fl_) motor_fl_->set_speed(output_left_duty_);
    if (motor_rl_) motor_rl_->set_speed(output_left_duty_);
    if (motor_fr_) motor_fr_->set_speed(output_right_duty_);
    if (motor_rr_) motor_rr_->set_speed(output_right_duty_);
}

void BaseController::stop_all() {
    target_left_vel_ = 0.0;
    target_right_vel_ = 0.0;
    output_left_duty_ = 0.0;
    output_right_duty_ = 0.0;

    if (motor_fl_) motor_fl_->stop();
    if (motor_fr_) motor_fr_->stop();
    if (motor_rl_) motor_rl_->stop();
    if (motor_rr_) motor_rr_->stop();

    pid_left_.reset();
    pid_right_.reset();
}

void BaseController::set_pid_gains(double kp, double ki, double kd,
                                    double max_output, double max_integral) {
    pid_left_.set_gains(kp, ki, kd);
    pid_left_.set_limits(max_output, max_integral);
    pid_right_.set_gains(kp, ki, kd);
    pid_right_.set_limits(max_output, max_integral);
}

bool BaseController::is_initialized() const {
    return motor_fl_ && motor_fl_->is_initialized() &&
           motor_fr_ && motor_fr_->is_initialized() &&
           motor_rl_ && motor_rl_->is_initialized() &&
           motor_rr_ && motor_rr_->is_initialized();
}
