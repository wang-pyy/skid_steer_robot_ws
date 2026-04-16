#pragma once

#include <cmath>

/**
 * 2D differential-drive odometry integrator.
 *
 * Integrates wheel velocities into (x, y, theta) pose.
 * Thread-safe reads are not needed here – called from a single timer thread.
 */
class DiffDriveOdometry {
public:
    DiffDriveOdometry(double track_width);

    /**
     * Update pose with new velocity measurements.
     * @param left_vel   Left-side linear velocity (m/s)
     * @param right_vel  Right-side linear velocity (m/s)
     * @param dt         Time step (seconds)
     */
    void update(double left_vel, double right_vel, double dt);

    /** Reset pose to origin. */
    void reset();

    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double linear_vel() const { return linear_vel_; }
    double angular_vel() const { return angular_vel_; }

private:
    double track_width_;
    double x_{0.0};
    double y_{0.0};
    double theta_{0.0};
    double linear_vel_{0.0};
    double angular_vel_{0.0};
};
