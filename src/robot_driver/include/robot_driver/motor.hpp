#pragma once

#include <gpiod.h>
#include <atomic>
#include <thread>
#include <string>

/**
 * Controls a single DC motor through an H-bridge driver (e.g. L298N, TB6612).
 *
 * Pin assignment:
 *   - in1_pin / in2_pin : direction control (IN1 HIGH + IN2 LOW = forward)
 *   - ena_pin           : enable / speed control via software PWM
 *
 * Software PWM runs on a dedicated thread toggling the ENA pin.
 * This approach works on any GPIO pin without requiring hardware PWM.
 *
 * NOTE on RPi 5: user-accessible GPIO is on /dev/gpiochip4 (RP1 chip).
 *       On RPi 4 it is /dev/gpiochip0.
 *       Configure via the gpio_chip parameter.
 */
class Motor {
public:
    /**
     * @param chip_path       GPIO chip device, e.g. "/dev/gpiochip4"
     * @param in1_pin         GPIO number for direction pin 1
     * @param in2_pin         GPIO number for direction pin 2
     * @param ena_pin         GPIO number for PWM / enable pin
     * @param pwm_freq_hz     Software PWM frequency (e.g. 1000 Hz)
     */
    Motor(const std::string& chip_path, int in1_pin, int in2_pin,
          int ena_pin, int pwm_freq_hz);
    ~Motor();

    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    /**
     * Set motor speed.
     * @param speed  -1.0 (full reverse) to +1.0 (full forward)
     */
    void set_speed(double speed);

    /** Immediately stop the motor (brake: both direction pins LOW) */
    void stop();

    /** Returns true if GPIO was successfully initialized */
    bool is_initialized() const { return initialized_; }

private:
    void pwm_loop();
    void set_direction(bool forward);
    void release_resources();

    struct gpiod_chip* chip_{nullptr};
    struct gpiod_line_request* dir_request_{nullptr};
    struct gpiod_line_request* ena_request_{nullptr};

    int in1_pin_, in2_pin_, ena_pin_;
    int pwm_freq_hz_;
    long period_ns_;                        // PWM period in nanoseconds
    std::atomic<double> duty_{0.0};         // 0.0 – 1.0
    std::atomic<bool> dir_forward_{true};
    std::atomic<bool> running_{true};
    bool initialized_{false};
    std::thread pwm_thread_;
};
