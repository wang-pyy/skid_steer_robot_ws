#include "robot_driver/yahboom_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace {

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) { out.push_back(cur); cur.clear(); }
        else          { cur.push_back(c); }
    }
    out.push_back(cur);
    return out;
}

}  // namespace

YahboomDriver::YahboomDriver(const Config& cfg) {
    if (!serial_.open(cfg.port, cfg.baudrate)) return;
    running_ = true;
    rx_thread_ = std::thread(&YahboomDriver::rx_loop, this);
}

YahboomDriver::~YahboomDriver() {
    running_ = false;
    serial_.close();           // unblocks read
    if (rx_thread_.joinable()) rx_thread_.join();
}

// ---- Commands ----

void YahboomDriver::send_frame(const std::string& body) {
    if (!serial_.is_open()) return;
    std::string frame = "$" + body + "#";
    serial_.write_all(frame);
}

void YahboomDriver::set_speed(int m1, int m2, int m3, int m4) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "spd:%d,%d,%d,%d", m1, m2, m3, m4);
    send_frame(buf);
}

void YahboomDriver::set_pwm(int m1, int m2, int m3, int m4) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "pwm:%d,%d,%d,%d", m1, m2, m3, m4);
    send_frame(buf);
}

void YahboomDriver::set_motor_type(int type) {
    char buf[32]; std::snprintf(buf, sizeof(buf), "mtype:%d", type);
    send_frame(buf);
}

void YahboomDriver::set_deadzone(int dz) {
    char buf[32]; std::snprintf(buf, sizeof(buf), "deadzone:%d", dz);
    send_frame(buf);
}

void YahboomDriver::set_encoder_lines(int lines) {
    char buf[32]; std::snprintf(buf, sizeof(buf), "mline:%d", lines);
    send_frame(buf);
}

void YahboomDriver::set_motor_phase(int phase) {
    char buf[32]; std::snprintf(buf, sizeof(buf), "mphase:%d", phase);
    send_frame(buf);
}

void YahboomDriver::set_wheel_diameter_mm(int mm) {
    char buf[32]; std::snprintf(buf, sizeof(buf), "wdiameter:%d", mm);
    send_frame(buf);
}

void YahboomDriver::set_upload(int mall, int mtep, int mspd) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "upload:%d,%d,%d", mall, mtep, mspd);
    send_frame(buf);
}

// ---- Feedback ----

YahboomDriver::Feedback YahboomDriver::feedback() const {
    std::lock_guard<std::mutex> lock(fb_mtx_);
    return latest_;
}

void YahboomDriver::set_feedback_callback(FeedbackCallback cb) {
    std::lock_guard<std::mutex> lock(fb_mtx_);
    fb_cb_ = std::move(cb);
}

void YahboomDriver::rx_loop() {
    std::string frame;
    while (running_.load()) {
        if (!serial_.read_frame(frame)) {
            if (!running_.load()) break;
            continue;   // timeout or transient error — keep looping
        }
        parse_frame(frame);
    }
}

void YahboomDriver::parse_frame(const std::string& body) {
    auto colon = body.find(':');
    if (colon == std::string::npos) return;
    std::string tag  = body.substr(0, colon);
    std::string data = body.substr(colon + 1);

    auto parts = split(data, ',');
    const auto now = std::chrono::steady_clock::now();

    Feedback snapshot;
    FeedbackCallback cb_copy;
    bool updated = false;

    {
        std::lock_guard<std::mutex> lock(fb_mtx_);

        if (tag == "MSPD" && parts.size() >= 4) {
            for (int i = 0; i < 4; ++i) latest_.speeds[i] = std::atof(parts[i].c_str());
            latest_.speed_stamp = now;
            latest_.has_speed   = true;
            updated = true;
        } else if (tag == "MTEP" && parts.size() >= 4) {
            for (int i = 0; i < 4; ++i) latest_.encoders[i] = std::atoi(parts[i].c_str());
            latest_.encoder_stamp = now;
            latest_.has_encoder   = true;
            updated = true;
        } else if (tag == "MAll" && parts.size() >= 8) {
            // Convention: first 4 = encoders, next 4 = speeds.
            // Adjust here if the board uses a different ordering.
            for (int i = 0; i < 4; ++i) latest_.encoders[i] = std::atoi(parts[i].c_str());
            for (int i = 0; i < 4; ++i) latest_.speeds[i]   = std::atof(parts[i + 4].c_str());
            latest_.speed_stamp   = now;
            latest_.encoder_stamp = now;
            latest_.has_speed     = true;
            latest_.has_encoder   = true;
            updated = true;
        }

        if (updated) {
            snapshot = latest_;
            cb_copy  = fb_cb_;
        }
    }

    if (updated && cb_copy) cb_copy(snapshot);
}
