#include "robot_driver/motor.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

// libgpiod v2 API
// If your system has libgpiod < 2.0, you will need to adapt.
// This code targets libgpiod >= 2.0 as available on Raspberry Pi OS Bookworm+.

Motor::Motor(const std::string& chip_path, int in1_pin, int in2_pin,
             int ena_pin, int pwm_freq_hz)
    : in1_pin_(in1_pin), in2_pin_(in2_pin), ena_pin_(ena_pin),
      pwm_freq_hz_(pwm_freq_hz) {

    period_ns_ = 1000000000L / pwm_freq_hz_;

    // Open GPIO chip
    chip_ = gpiod_chip_open(chip_path.c_str());
    if (!chip_) {
        return;  // initialized_ remains false
    }

    // -- Configure direction pins (IN1, IN2) as outputs --
    struct gpiod_line_settings* dir_settings = gpiod_line_settings_new();
    if (!dir_settings) { release_resources(); return; }
    gpiod_line_settings_set_direction(dir_settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(dir_settings, GPIOD_LINE_VALUE_INACTIVE);

    struct gpiod_line_config* dir_cfg = gpiod_line_config_new();
    if (!dir_cfg) { gpiod_line_settings_free(dir_settings); release_resources(); return; }

    unsigned int dir_offsets[2] = {
        static_cast<unsigned int>(in1_pin_),
        static_cast<unsigned int>(in2_pin_)
    };
    int ret = gpiod_line_config_add_line_settings(dir_cfg, dir_offsets, 2, dir_settings);
    gpiod_line_settings_free(dir_settings);
    if (ret < 0) { gpiod_line_config_free(dir_cfg); release_resources(); return; }

    struct gpiod_request_config* dir_req_cfg = gpiod_request_config_new();
    if (!dir_req_cfg) { gpiod_line_config_free(dir_cfg); release_resources(); return; }
    gpiod_request_config_set_consumer(dir_req_cfg, "robot_driver_dir");

    dir_request_ = gpiod_chip_request_lines(chip_, dir_req_cfg, dir_cfg);
    gpiod_request_config_free(dir_req_cfg);
    gpiod_line_config_free(dir_cfg);
    if (!dir_request_) { release_resources(); return; }

    // -- Configure ENA pin as output --
    struct gpiod_line_settings* ena_settings = gpiod_line_settings_new();
    if (!ena_settings) { release_resources(); return; }
    gpiod_line_settings_set_direction(ena_settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(ena_settings, GPIOD_LINE_VALUE_INACTIVE);

    struct gpiod_line_config* ena_cfg = gpiod_line_config_new();
    if (!ena_cfg) { gpiod_line_settings_free(ena_settings); release_resources(); return; }

    unsigned int ena_offset = static_cast<unsigned int>(ena_pin_);
    ret = gpiod_line_config_add_line_settings(ena_cfg, &ena_offset, 1, ena_settings);
    gpiod_line_settings_free(ena_settings);
    if (ret < 0) { gpiod_line_config_free(ena_cfg); release_resources(); return; }

    struct gpiod_request_config* ena_req_cfg = gpiod_request_config_new();
    if (!ena_req_cfg) { gpiod_line_config_free(ena_cfg); release_resources(); return; }
    gpiod_request_config_set_consumer(ena_req_cfg, "robot_driver_ena");

    ena_request_ = gpiod_chip_request_lines(chip_, ena_req_cfg, ena_cfg);
    gpiod_request_config_free(ena_req_cfg);
    gpiod_line_config_free(ena_cfg);
    if (!ena_request_) { release_resources(); return; }

    initialized_ = true;

    // Start software PWM thread
    pwm_thread_ = std::thread(&Motor::pwm_loop, this);
}

Motor::~Motor() {
    stop();
    running_ = false;
    if (pwm_thread_.joinable()) {
        pwm_thread_.join();
    }
    // Ensure pins are LOW on exit
    if (dir_request_) {
        gpiod_line_request_set_value(dir_request_, in1_pin_, GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_set_value(dir_request_, in2_pin_, GPIOD_LINE_VALUE_INACTIVE);
    }
    if (ena_request_) {
        gpiod_line_request_set_value(ena_request_, ena_pin_, GPIOD_LINE_VALUE_INACTIVE);
    }
    release_resources();
}

void Motor::set_speed(double speed) {
    if (speed >= 0.0) {
        dir_forward_ = true;
        duty_ = std::min(std::abs(speed), 1.0);
    } else {
        dir_forward_ = false;
        duty_ = std::min(std::abs(speed), 1.0);
    }
    set_direction(dir_forward_);
}

void Motor::stop() {
    duty_ = 0.0;
    if (dir_request_) {
        gpiod_line_request_set_value(dir_request_, in1_pin_, GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_set_value(dir_request_, in2_pin_, GPIOD_LINE_VALUE_INACTIVE);
    }
    if (ena_request_) {
        gpiod_line_request_set_value(ena_request_, ena_pin_, GPIOD_LINE_VALUE_INACTIVE);
    }
}

void Motor::set_direction(bool forward) {
    if (!dir_request_) return;
    if (forward) {
        gpiod_line_request_set_value(dir_request_, in1_pin_, GPIOD_LINE_VALUE_ACTIVE);
        gpiod_line_request_set_value(dir_request_, in2_pin_, GPIOD_LINE_VALUE_INACTIVE);
    } else {
        gpiod_line_request_set_value(dir_request_, in1_pin_, GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_set_value(dir_request_, in2_pin_, GPIOD_LINE_VALUE_ACTIVE);
    }
}

void Motor::pwm_loop() {
    while (running_) {
        double d = duty_.load();

        if (d < 0.001) {
            // Fully off – sleep a bit to save CPU
            if (ena_request_) {
                gpiod_line_request_set_value(ena_request_, ena_pin_, GPIOD_LINE_VALUE_INACTIVE);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (d > 0.999) {
            // Fully on
            if (ena_request_) {
                gpiod_line_request_set_value(ena_request_, ena_pin_, GPIOD_LINE_VALUE_ACTIVE);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // PWM cycle
        long on_ns  = static_cast<long>(period_ns_ * d);
        long off_ns = period_ns_ - on_ns;

        if (ena_request_) {
            gpiod_line_request_set_value(ena_request_, ena_pin_, GPIOD_LINE_VALUE_ACTIVE);
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(on_ns));

        if (ena_request_) {
            gpiod_line_request_set_value(ena_request_, ena_pin_, GPIOD_LINE_VALUE_INACTIVE);
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(off_ns));
    }
}

void Motor::release_resources() {
    if (dir_request_) {
        gpiod_line_request_release(dir_request_);
        dir_request_ = nullptr;
    }
    if (ena_request_) {
        gpiod_line_request_release(ena_request_);
        ena_request_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}
