#pragma once

#include "robot_driver/serial_port.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

/**
 * Yahboom 4-channel motor driver — ASCII serial protocol.
 *
 * Frame format: everything is delimited by '$' ... '#'.
 *
 * Commands (host → board):
 *   $spd:m1,m2,m3,m4#          target speed for each channel (mm/s)
 *   $pwm:m1,m2,m3,m4#          raw PWM per channel
 *   $mtype:x#                  motor type
 *   $deadzone:x#               PWM deadzone
 *   $mline:x#                  encoder lines per revolution
 *   $mphase:x#                 motor phase / direction
 *   $wdiameter:x#              wheel diameter (mm)
 *   $upload:a,b,c#             upload flags: (MAll, MTEP, MSPD)
 *
 * Feedback (board → host):
 *   $MAll:...#                 combined status (optional)
 *   $MTEP:e1,e2,e3,e4#         encoder ticks per channel
 *   $MSPD:s1,s2,s3,s4#         measured speed per channel (mm/s)
 */
class YahboomDriver {
public:
    struct Config {
        std::string port{"/dev/ttyUSB0"};
        int baudrate{115200};
    };

    struct Feedback {
        std::array<double, 4>  speeds{{0.0, 0.0, 0.0, 0.0}};  // mm/s
        std::array<int32_t, 4> encoders{{0, 0, 0, 0}};         // ticks
        std::chrono::steady_clock::time_point speed_stamp{};
        std::chrono::steady_clock::time_point encoder_stamp{};
        bool has_speed{false};
        bool has_encoder{false};
    };

    using FeedbackCallback = std::function<void(const Feedback&)>;

    explicit YahboomDriver(const Config& cfg);
    ~YahboomDriver();

    YahboomDriver(const YahboomDriver&) = delete;
    YahboomDriver& operator=(const YahboomDriver&) = delete;

    bool is_open() const { return serial_.is_open(); }

    // ---- Commands ----
    void set_speed(int m1, int m2, int m3, int m4);
    void set_pwm(int m1, int m2, int m3, int m4);
    void set_motor_type(int type);
    void set_deadzone(int dz);
    void set_encoder_lines(int lines);
    void set_motor_phase(int phase);
    void set_wheel_diameter_mm(int mm);
    void set_upload(int mall, int mtep, int mspd);

    /** Latest parsed feedback (thread-safe copy). */
    Feedback feedback() const;

    /** Optional callback invoked on every parsed feedback frame. */
    void set_feedback_callback(FeedbackCallback cb);

private:
    void rx_loop();
    void parse_frame(const std::string& body);
    void send_frame(const std::string& body);

    SerialPort serial_;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex fb_mtx_;
    Feedback latest_{};
    FeedbackCallback fb_cb_;
};
