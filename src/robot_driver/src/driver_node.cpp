#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

#include "robot_driver/base_controller.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <csignal>

/**
 * ROS 2 node for skid-steer robot motor control.
 *
 * Subscribes:
 *   /cmd_vel              (geometry_msgs/Twist)  – velocity command
 *   /wheel/left_vel       (std_msgs/Float64)     – measured left velocity for PID
 *   /wheel/right_vel      (std_msgs/Float64)     – measured right velocity for PID
 *
 * Publishes (debug):
 *   /driver/left_target   (std_msgs/Float64)
 *   /driver/right_target  (std_msgs/Float64)
 *   /driver/left_pwm      (std_msgs/Float64)
 *   /driver/right_pwm     (std_msgs/Float64)
 *
 * Safety features:
 *   - cmd_vel timeout (default 0.5 s) → auto-stop
 *   - SIGINT / node shutdown → all motors stop
 */
class DriverNode : public rclcpp::Node {
public:
    DriverNode()
        : Node("driver_node") {
        declare_parameters();
        setup_controller();
        setup_ros();

        RCLCPP_INFO(get_logger(),
                     "Driver node started. open_loop=%s, freq=%.1f Hz",
                     open_loop_ ? "true" : "false", control_freq_);
    }

    ~DriverNode() override {
        if (controller_) {
            controller_->stop_all();
        }
        RCLCPP_INFO(get_logger(), "Driver node shut down – motors stopped.");
    }

private:
    // ---- Parameter declaration ----
    void declare_parameters() {
        declare_parameter<std::string>("gpio_chip", "/dev/gpiochip4");

        // Motor FL pins
        declare_parameter<int>("motor_fl_in1", 17);
        declare_parameter<int>("motor_fl_in2", 27);
        declare_parameter<int>("motor_fl_ena", 22);
        // Motor FR pins
        declare_parameter<int>("motor_fr_in1", 23);
        declare_parameter<int>("motor_fr_in2", 24);
        declare_parameter<int>("motor_fr_ena", 25);
        // Motor RL pins
        declare_parameter<int>("motor_rl_in1", 5);
        declare_parameter<int>("motor_rl_in2", 6);
        declare_parameter<int>("motor_rl_ena", 13);
        // Motor RR pins
        declare_parameter<int>("motor_rr_in1", 16);
        declare_parameter<int>("motor_rr_in2", 20);
        declare_parameter<int>("motor_rr_ena", 21);

        declare_parameter<int>("pwm_frequency", 1000);
        declare_parameter<double>("max_duty_cycle", 0.95);
        declare_parameter<double>("wheel_radius", 0.033);
        declare_parameter<double>("track_width", 0.23);
        declare_parameter<double>("max_linear_speed", 0.5);
        declare_parameter<double>("max_angular_speed", 2.0);

        declare_parameter<double>("pid_kp", 1.0);
        declare_parameter<double>("pid_ki", 0.5);
        declare_parameter<double>("pid_kd", 0.01);
        declare_parameter<double>("pid_max_output", 1.0);
        declare_parameter<double>("pid_max_integral", 0.5);

        declare_parameter<double>("control_frequency", 50.0);
        declare_parameter<double>("cmd_vel_timeout", 0.5);
        declare_parameter<bool>("open_loop", false);
    }

    // ---- Build BaseController from parameters ----
    void setup_controller() {
        BaseController::Config cfg;
        cfg.gpio_chip        = get_parameter("gpio_chip").as_string();
        cfg.fl               = {get_parameter("motor_fl_in1").as_int(),
                                get_parameter("motor_fl_in2").as_int(),
                                get_parameter("motor_fl_ena").as_int()};
        cfg.fr               = {get_parameter("motor_fr_in1").as_int(),
                                get_parameter("motor_fr_in2").as_int(),
                                get_parameter("motor_fr_ena").as_int()};
        cfg.rl               = {get_parameter("motor_rl_in1").as_int(),
                                get_parameter("motor_rl_in2").as_int(),
                                get_parameter("motor_rl_ena").as_int()};
        cfg.rr               = {get_parameter("motor_rr_in1").as_int(),
                                get_parameter("motor_rr_in2").as_int(),
                                get_parameter("motor_rr_ena").as_int()};
        cfg.pwm_frequency    = get_parameter("pwm_frequency").as_int();
        cfg.max_duty_cycle   = get_parameter("max_duty_cycle").as_double();
        cfg.wheel_radius     = get_parameter("wheel_radius").as_double();
        cfg.track_width      = get_parameter("track_width").as_double();
        cfg.max_linear_speed = get_parameter("max_linear_speed").as_double();
        cfg.max_angular_speed= get_parameter("max_angular_speed").as_double();
        cfg.open_loop        = get_parameter("open_loop").as_bool();
        open_loop_           = cfg.open_loop;
        control_freq_        = get_parameter("control_frequency").as_double();

        controller_ = std::make_unique<BaseController>(cfg);

        double kp = get_parameter("pid_kp").as_double();
        double ki = get_parameter("pid_ki").as_double();
        double kd = get_parameter("pid_kd").as_double();
        double max_out = get_parameter("pid_max_output").as_double();
        double max_int = get_parameter("pid_max_integral").as_double();
        controller_->set_pid_gains(kp, ki, kd, max_out, max_int);

        if (!controller_->is_initialized()) {
            RCLCPP_ERROR(get_logger(),
                "One or more motors failed to initialize GPIO. "
                "Check gpio_chip path and pin numbers. "
                "Ensure the user has permission to access GPIO "
                "(e.g., member of 'gpio' group or running as root).");
        }
    }

    // ---- ROS subscriptions / publishers / timers ----
    void setup_ros() {
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                controller_->set_target(msg->linear.x, msg->angular.z);
                last_cmd_time_ = now();
            });

        left_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
            "wheel/left_vel", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                measured_left_ = msg->data;
            });

        right_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
            "wheel/right_vel", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                measured_right_ = msg->data;
            });

        // Debug publishers
        pub_left_target_  = create_publisher<std_msgs::msg::Float64>("driver/left_target", 10);
        pub_right_target_ = create_publisher<std_msgs::msg::Float64>("driver/right_target", 10);
        pub_left_pwm_     = create_publisher<std_msgs::msg::Float64>("driver/left_pwm", 10);
        pub_right_pwm_    = create_publisher<std_msgs::msg::Float64>("driver/right_pwm", 10);

        double period_ms = 1000.0 / control_freq_;
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&DriverNode::control_callback, this));

        last_cmd_time_ = now();
    }

    // ---- Main control loop ----
    void control_callback() {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check cmd_vel timeout
        double timeout_sec = get_parameter("cmd_vel_timeout").as_double();
        auto elapsed = (now() - last_cmd_time_).seconds();
        if (elapsed > timeout_sec) {
            controller_->stop_all();
            return;
        }

        double dt = 1.0 / control_freq_;
        controller_->update(measured_left_, measured_right_, dt);

        // Publish debug info
        auto msg = std_msgs::msg::Float64();
        msg.data = controller_->target_left_vel();
        pub_left_target_->publish(msg);
        msg.data = controller_->target_right_vel();
        pub_right_target_->publish(msg);
        msg.data = controller_->output_left_duty();
        pub_left_pwm_->publish(msg);
        msg.data = controller_->output_right_duty();
        pub_right_pwm_->publish(msg);
    }

    // ---- Members ----
    std::unique_ptr<BaseController> controller_;
    std::mutex mutex_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_vel_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_pwm_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_pwm_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_cmd_time_;

    double measured_left_{0.0};
    double measured_right_{0.0};
    double control_freq_{50.0};
    bool open_loop_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
