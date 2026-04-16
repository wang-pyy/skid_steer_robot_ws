#pragma once

#include "robot_driver/motor.hpp"
#include "robot_driver/pid_controller.hpp"
#include <memory>
#include <string>

/**
 * Differential-drive base controller for a 4-wheel skid-steer robot.
 *
 * Kinematic model:
 *   v_left  = linear_x - angular_z * track_width / 2
 *   v_right = linear_x + angular_z * track_width / 2
 *
 * Each side has two motors that receive the same command.
 * PID controllers close the loop using measured wheel velocities.
 * In open-loop mode, velocity commands are mapped directly to duty cycle.
 */
class BaseController {
public:
    struct MotorPins {
        int in1;
        int in2;
        int ena;
    };

    struct Config {
        std::string gpio_chip;
        MotorPins fl, fr, rl, rr;
        int pwm_frequency;
        double max_duty_cycle;
        double wheel_radius;
        double track_width;
        double max_linear_speed;
        double max_angular_speed;
        bool open_loop;
    };

    explicit BaseController(const Config& cfg);
    ~BaseController();

    BaseController(const BaseController&) = delete;
    BaseController& operator=(const BaseController&) = delete;

    /**
     * Set target velocities from Twist message.
     * @param linear_x   Forward speed (m/s)
     * @param angular_z  Yaw rate (rad/s)
     */
    void set_target(double linear_x, double angular_z);

    /**
     * Update control loop with measured wheel velocities.
     * Call this at the control frequency.
     * @param measured_left   Left-side average wheel speed (m/s)
     * @param measured_right  Right-side average wheel speed (m/s)
     * @param dt              Time step (seconds)
     */
    void update(double measured_left, double measured_right, double dt);

    /** Emergency stop – all motors off, PID reset */
    void stop_all();

    /** Set PID gains for both sides */
    void set_pid_gains(double kp, double ki, double kd,
                       double max_output, double max_integral);

    /** Check if all motors initialized */
    bool is_initialized() const;

    // Getters for debug topics
    double target_left_vel() const { return target_left_vel_; }
    double target_right_vel() const { return target_right_vel_; }
    double output_left_duty() const { return output_left_duty_; }
    double output_right_duty() const { return output_right_duty_; }

private:
    Config cfg_;

    std::unique_ptr<Motor> motor_fl_;
    std::unique_ptr<Motor> motor_fr_;
    std::unique_ptr<Motor> motor_rl_;
    std::unique_ptr<Motor> motor_rr_;

    PIDController pid_left_;
    PIDController pid_right_;

    double target_left_vel_{0.0};
    double target_right_vel_{0.0};
    double output_left_duty_{0.0};
    double output_right_duty_{0.0};
};
