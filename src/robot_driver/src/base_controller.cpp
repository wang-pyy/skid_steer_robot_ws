#include "robot_driver/base_controller.hpp"

#include <algorithm>
#include <cmath>

BaseController::BaseController(const Config& cfg)
    : cfg_(cfg) {
    YahboomDriver::Config dcfg;
    dcfg.port     = cfg_.port;
    dcfg.baudrate = cfg_.baudrate;
    driver_ = std::make_unique<YahboomDriver>(dcfg);
}

BaseController::~BaseController() {
    stop_all();
}

void BaseController::set_target(double linear_x, double angular_z) {
    linear_x  = std::clamp(linear_x,  -cfg_.max_linear_speed,  cfg_.max_linear_speed);
    angular_z = std::clamp(angular_z, -cfg_.max_angular_speed, cfg_.max_angular_speed);

    target_left_vel_  = linear_x - angular_z * cfg_.track_width / 2.0;
    target_right_vel_ = linear_x + angular_z * cfg_.track_width / 2.0;

    double max_vel = std::max(std::abs(target_left_vel_), std::abs(target_right_vel_));
    if (max_vel > cfg_.max_linear_speed) {
        double scale = cfg_.max_linear_speed / max_vel;
        target_left_vel_  *= scale;
        target_right_vel_ *= scale;
    }
}

std::array<int, 4> BaseController::to_channels(int left_value, int right_value) const {
    std::array<int, 4> ch{{0, 0, 0, 0}};
    auto assign = [&](int channel, int value) {
        if (channel >= 1 && channel <= 4) ch[channel - 1] = value;
    };
    int left  = cfg_.invert_left  ? -left_value  : left_value;
    int right = cfg_.invert_right ? -right_value : right_value;
    assign(cfg_.channels.fl, left);
    assign(cfg_.channels.rl, left);
    assign(cfg_.channels.fr, right);
    assign(cfg_.channels.rr, right);
    return ch;
}

void BaseController::update(double dt) {
    if (!driver_ || !driver_->is_open()) return;

    // Pull latest measured speeds (mm/s from board) averaged per side.
    auto fb = driver_->feedback();
    auto pick = [&](int ch) -> double {
        if (ch < 1 || ch > 4) return 0.0;
        return fb.speeds[ch - 1] / 1000.0;          // mm/s → m/s
    };
    measured_left_vel_  = (pick(cfg_.channels.fl) + pick(cfg_.channels.rl)) / 2.0;
    measured_right_vel_ = (pick(cfg_.channels.fr) + pick(cfg_.channels.rr)) / 2.0;
    if (cfg_.invert_left)  measured_left_vel_  = -measured_left_vel_;
    if (cfg_.invert_right) measured_right_vel_ = -measured_right_vel_;

    if (cfg_.mode == ControlMode::Speed) {
        // Send m/s target as mm/s integers; board handles closed loop.
        int left_mms  = static_cast<int>(std::lround(target_left_vel_  * 1000.0));
        int right_mms = static_cast<int>(std::lround(target_right_vel_ * 1000.0));
        auto ch = to_channels(left_mms, right_mms);
        driver_->set_speed(ch[0], ch[1], ch[2], ch[3]);

        output_left_duty_  = target_left_vel_  / cfg_.max_linear_speed;
        output_right_duty_ = target_right_vel_ / cfg_.max_linear_speed;
    } else {
        // PID on host: setpoint and measured are both m/s.
        double u_left  = pid_left_.compute(target_left_vel_,  measured_left_vel_,  dt);
        double u_right = pid_right_.compute(target_right_vel_, measured_right_vel_, dt);
        u_left  = std::clamp(u_left,  -1.0, 1.0);
        u_right = std::clamp(u_right, -1.0, 1.0);
        output_left_duty_  = u_left;
        output_right_duty_ = u_right;

        int pwm_left  = static_cast<int>(std::lround(u_left  * cfg_.pwm_max));
        int pwm_right = static_cast<int>(std::lround(u_right * cfg_.pwm_max));
        auto ch = to_channels(pwm_left, pwm_right);
        driver_->set_pwm(ch[0], ch[1], ch[2], ch[3]);
    }
}

void BaseController::stop_all() {
    target_left_vel_   = 0.0;
    target_right_vel_  = 0.0;
    output_left_duty_  = 0.0;
    output_right_duty_ = 0.0;

    if (driver_ && driver_->is_open()) {
        driver_->set_speed(0, 0, 0, 0);
        driver_->set_pwm(0, 0, 0, 0);
    }
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
