#pragma once

/**
 * Simple PID controller with anti-windup and output clamping.
 * Designed for motor speed control in a differential-drive robot.
 */
class PIDController {
public:
    PIDController() = default;
    PIDController(double kp, double ki, double kd,
                  double max_output, double max_integral);

    /**
     * Compute PID output.
     * @param setpoint  Desired value
     * @param measured  Current measured value
     * @param dt        Time step in seconds
     * @return Control output clamped to [-max_output, max_output]
     */
    double compute(double setpoint, double measured, double dt);

    /** Reset integral and derivative state */
    void reset();

    /** Update gains at runtime */
    void set_gains(double kp, double ki, double kd);

    void set_limits(double max_output, double max_integral);

private:
    double kp_{0.0};
    double ki_{0.0};
    double kd_{0.0};
    double max_output_{1.0};
    double max_integral_{1.0};

    double integral_{0.0};
    double prev_error_{0.0};
    bool first_run_{true};
};
