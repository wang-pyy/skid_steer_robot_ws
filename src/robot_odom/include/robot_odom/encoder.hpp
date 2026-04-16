#pragma once

#include <gpiod.h>
#include <atomic>
#include <thread>
#include <string>

/**
 * Reads a quadrature (or single-phase) rotary encoder via GPIO edge events.
 *
 * Quadrature mode (A+B):
 *   Monitors both edges on channel A and reads channel B to determine direction.
 *   Resolution: 4× counts per encoder pulse (full quadrature decode).
 *
 * Single-phase mode (A only):
 *   Counts rising edges on channel A. No direction information –
 *   always increments. The odometry node must infer direction from
 *   the motor command sign.
 *
 * Thread-safe: tick counter is std::atomic<int64_t>.
 */
class Encoder {
public:
    /**
     * @param chip_path       GPIO chip, e.g. "/dev/gpiochip4"
     * @param pin_a           GPIO number for encoder channel A
     * @param pin_b           GPIO number for encoder channel B (ignored in single-phase)
     * @param single_phase    true = count A rising edges only
     * @param counts_per_rev  Encoder PPR on the motor shaft (before gear reduction)
     * @param gear_ratio      Gear ratio (motor shaft turns per wheel turn)
     */
    Encoder(const std::string& chip_path, int pin_a, int pin_b,
            bool single_phase, int counts_per_rev, double gear_ratio);
    ~Encoder();

    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;

    /** Accumulated tick count (thread-safe). */
    int64_t get_ticks() const { return ticks_.load(std::memory_order_relaxed); }

    /** Reset counter to zero. */
    void reset() { ticks_.store(0, std::memory_order_relaxed); }

    /** True if GPIO setup succeeded. */
    bool is_initialized() const { return initialized_; }

    /** Ticks per full wheel revolution (accounts for gear ratio and quadrature). */
    double ticks_per_wheel_rev() const;

private:
    void counting_loop();
    void release_resources();

    struct gpiod_chip* chip_{nullptr};
    struct gpiod_line_request* line_req_a_{nullptr};  // edge events on A
    struct gpiod_line_request* line_req_b_{nullptr};  // input-only for B (quadrature)

    int pin_a_, pin_b_;
    bool single_phase_;
    int counts_per_rev_;
    double gear_ratio_;

    std::atomic<int64_t> ticks_{0};
    std::atomic<bool> running_{true};
    bool initialized_{false};
    std::thread thread_;
};
