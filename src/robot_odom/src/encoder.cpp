#include "robot_odom/encoder.hpp"
#include <cstring>
#include <cmath>

// Minimum interval between edges to accept (debounce), in nanoseconds.
// 50 µs = 50000 ns → max ~20 kHz pulse rate, well above any small DC motor.
static constexpr int64_t DEBOUNCE_NS = 50000;

Encoder::Encoder(const std::string& chip_path, int pin_a, int pin_b,
                 bool single_phase, int counts_per_rev, double gear_ratio)
    : pin_a_(pin_a), pin_b_(pin_b), single_phase_(single_phase),
      counts_per_rev_(counts_per_rev), gear_ratio_(gear_ratio) {

    // ---- Open chip ----
    chip_ = gpiod_chip_open(chip_path.c_str());
    if (!chip_) return;

    // ---- Channel A: edge detection (both edges for quadrature, rising for single) ----
    struct gpiod_line_settings* settings_a = gpiod_line_settings_new();
    if (!settings_a) { release_resources(); return; }

    gpiod_line_settings_set_direction(settings_a, GPIOD_LINE_DIRECTION_INPUT);
    if (single_phase_) {
        gpiod_line_settings_set_edge_detection(settings_a, GPIOD_LINE_EDGE_RISING);
    } else {
        gpiod_line_settings_set_edge_detection(settings_a, GPIOD_LINE_EDGE_BOTH);
    }
    // Use internal pull-up if the encoder has open-collector output
    gpiod_line_settings_set_bias(settings_a, GPIOD_LINE_BIAS_PULL_UP);
    gpiod_line_settings_set_debounce_period_us(settings_a, static_cast<unsigned long>(DEBOUNCE_NS / 1000));

    struct gpiod_line_config* cfg_a = gpiod_line_config_new();
    if (!cfg_a) { gpiod_line_settings_free(settings_a); release_resources(); return; }

    unsigned int offset_a = static_cast<unsigned int>(pin_a_);
    if (gpiod_line_config_add_line_settings(cfg_a, &offset_a, 1, settings_a) < 0) {
        gpiod_line_settings_free(settings_a);
        gpiod_line_config_free(cfg_a);
        release_resources();
        return;
    }
    gpiod_line_settings_free(settings_a);

    struct gpiod_request_config* req_cfg_a = gpiod_request_config_new();
    if (!req_cfg_a) { gpiod_line_config_free(cfg_a); release_resources(); return; }
    gpiod_request_config_set_consumer(req_cfg_a, "encoder_a");

    line_req_a_ = gpiod_chip_request_lines(chip_, req_cfg_a, cfg_a);
    gpiod_request_config_free(req_cfg_a);
    gpiod_line_config_free(cfg_a);
    if (!line_req_a_) { release_resources(); return; }

    // ---- Channel B: plain input (only needed for quadrature) ----
    if (!single_phase_) {
        struct gpiod_line_settings* settings_b = gpiod_line_settings_new();
        if (!settings_b) { release_resources(); return; }
        gpiod_line_settings_set_direction(settings_b, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_bias(settings_b, GPIOD_LINE_BIAS_PULL_UP);

        struct gpiod_line_config* cfg_b = gpiod_line_config_new();
        if (!cfg_b) { gpiod_line_settings_free(settings_b); release_resources(); return; }

        unsigned int offset_b = static_cast<unsigned int>(pin_b_);
        if (gpiod_line_config_add_line_settings(cfg_b, &offset_b, 1, settings_b) < 0) {
            gpiod_line_settings_free(settings_b);
            gpiod_line_config_free(cfg_b);
            release_resources();
            return;
        }
        gpiod_line_settings_free(settings_b);

        struct gpiod_request_config* req_cfg_b = gpiod_request_config_new();
        if (!req_cfg_b) { gpiod_line_config_free(cfg_b); release_resources(); return; }
        gpiod_request_config_set_consumer(req_cfg_b, "encoder_b");

        line_req_b_ = gpiod_chip_request_lines(chip_, req_cfg_b, cfg_b);
        gpiod_request_config_free(req_cfg_b);
        gpiod_line_config_free(cfg_b);
        if (!line_req_b_) { release_resources(); return; }
    }

    initialized_ = true;
    thread_ = std::thread(&Encoder::counting_loop, this);
}

Encoder::~Encoder() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    release_resources();
}

double Encoder::ticks_per_wheel_rev() const {
    // For quadrature mode: each motor revolution produces counts_per_rev * 4 edges.
    // For single-phase: counts_per_rev rising edges per motor rev.
    // One wheel rev = gear_ratio motor revs.
    double motor_ticks = single_phase_
        ? static_cast<double>(counts_per_rev_)
        : static_cast<double>(counts_per_rev_) * 4.0;
    return motor_ticks * gear_ratio_;
}

void Encoder::counting_loop() {
    // Buffer for edge events
    struct gpiod_edge_event_buffer* event_buf = gpiod_edge_event_buffer_new(16);
    if (!event_buf) return;

    while (running_) {
        // Wait for edge events with 100 ms timeout so we can check running_ flag
        int ret = gpiod_line_request_wait_edge_events(line_req_a_, 100000000);  // 100 ms in ns
        if (ret < 0) {
            // Error
            break;
        }
        if (ret == 0) {
            // Timeout, no events – just loop and check running_
            continue;
        }

        // Read available events
        int num = gpiod_line_request_read_edge_events(line_req_a_, event_buf, 16);
        if (num < 0) break;

        for (int i = 0; i < num; i++) {
            struct gpiod_edge_event* event = gpiod_edge_event_buffer_get_event(event_buf, i);

            if (single_phase_) {
                // Single phase: just count up
                ticks_.fetch_add(1, std::memory_order_relaxed);
            } else {
                // Quadrature decode:
                // Read current B state to determine direction.
                enum gpiod_line_value b_val = gpiod_line_request_get_value(
                    line_req_b_, static_cast<unsigned int>(pin_b_));
                enum gpiod_edge_event_type edge_type = gpiod_edge_event_get_event_type(event);

                bool a_rising = (edge_type == GPIOD_EDGE_EVENT_RISING_EDGE);
                bool b_high = (b_val == GPIOD_LINE_VALUE_ACTIVE);

                // Standard quadrature truth table:
                //   A rising  + B low  → forward (+1)
                //   A rising  + B high → reverse (-1)
                //   A falling + B high → forward (+1)
                //   A falling + B low  → reverse (-1)
                int dir;
                if (a_rising) {
                    dir = b_high ? -1 : 1;
                } else {
                    dir = b_high ? 1 : -1;
                }
                ticks_.fetch_add(dir, std::memory_order_relaxed);
            }
        }
    }

    gpiod_edge_event_buffer_free(event_buf);
}

void Encoder::release_resources() {
    if (line_req_a_) {
        gpiod_line_request_release(line_req_a_);
        line_req_a_ = nullptr;
    }
    if (line_req_b_) {
        gpiod_line_request_release(line_req_b_);
        line_req_b_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}
