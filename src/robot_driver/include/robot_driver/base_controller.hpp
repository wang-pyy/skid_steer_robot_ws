#pragma once

#include "robot_driver/pid_controller.hpp"
#include "robot_driver/yahboom_driver.hpp"

#include <array>
#include <memory>
#include <string>

/**
 * Differential-drive base controller for a 4-wheel skid-steer robot driven by
 * a Yahboom 4-channel motor-driver board over USB serial.
 *
 * Kinematics:
 *   v_left  = linear_x - angular_z * track_width / 2
 *   v_right = linear_x + angular_z * track_width / 2
 *
 * Two control modes:
 *   - "spd": send target wheel speeds via $spd:...# and let the board close
 *           the loop internally (no ROS-side PID).
 *   - "pwm": run PID on the host using MSPD feedback and send $pwm:...#.
 */
class BaseController {
public:
    enum class ControlMode { Speed, Pwm };

    /** Which Yahboom channel (1..4) drives each physical wheel. */
    struct ChannelMap {
        int fl{1};
        int fr{2};
        int rl{3};
        int rr{4};
    };

    struct Config {
        std::string port{"/dev/ttyUSB0"};
        int baudrate{115200};

        ChannelMap channels;
        bool invert_left{false};
        bool invert_right{false};

        ControlMode mode{ControlMode::Speed};
        int pwm_max{1000};                 // output range for $pwm: [-pwm_max, +pwm_max]

        double wheel_radius{0.033};
        double track_width{0.23};
        double max_linear_speed{0.5};      // m/s
        double max_angular_speed{2.0};     // rad/s
    };

    explicit BaseController(const Config& cfg);
    ~BaseController();

    BaseController(const BaseController&) = delete;
    BaseController& operator=(const BaseController&) = delete;

    /** Access to underlying driver for one-shot config commands / feedback. */
    YahboomDriver& driver() { return *driver_; }

    /** Set target velocities from Twist. */
    void set_target(double linear_x, double angular_z);

    /** Run one control tick. @p dt in seconds. */
    void update(double dt);

    /** Emergency stop — all channels to 0, PID reset. */
    void stop_all();

    void set_pid_gains(double kp, double ki, double kd,
                       double max_output, double max_integral);

    bool is_initialized() const { return driver_ && driver_->is_open(); }

    // Debug getters
    double target_left_vel()  const { return target_left_vel_; }
    double target_right_vel() const { return target_right_vel_; }
    double output_left_duty()  const { return output_left_duty_; }
    double output_right_duty() const { return output_right_duty_; }
    double measured_left_vel()  const { return measured_left_vel_; }
    double measured_right_vel() const { return measured_right_vel_; }

private:
    /** Fill 4-element channel array from left/right command. */
    std::array<int, 4> to_channels(int left_value, int right_value) const;

    Config cfg_;
    std::unique_ptr<YahboomDriver> driver_;

    PIDController pid_left_;
    PIDController pid_right_;

    double target_left_vel_{0.0};
    double target_right_vel_{0.0};
    double output_left_duty_{0.0};
    double output_right_duty_{0.0};
    double measured_left_vel_{0.0};
    double measured_right_vel_{0.0};
};
