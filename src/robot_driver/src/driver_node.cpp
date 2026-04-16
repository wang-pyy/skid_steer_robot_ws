#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "robot_driver/base_controller.hpp"

#include <chrono>
#include <memory>
#include <mutex>

/**
 * ROS 2 node for a skid-steer robot driven by a Yahboom 4-channel USB-serial
 * motor-driver board.
 *
 * Subscribes:
 *   /cmd_vel              (geometry_msgs/Twist)       – velocity command
 *
 * Publishes:
 *   /driver/left_target   (std_msgs/Float64)          – target left m/s
 *   /driver/right_target  (std_msgs/Float64)          – target right m/s
 *   /driver/left_output   (std_msgs/Float64)          – control output [-1,1]
 *   /driver/right_output  (std_msgs/Float64)          – control output [-1,1]
 *   /driver/motor_speeds  (std_msgs/Float64MultiArray)– per-channel speed (mm/s)
 *   /driver/motor_encoders(std_msgs/Int32MultiArray)  – per-channel ticks
 *
 * Safety:
 *   - cmd_vel timeout (default 0.5 s) → auto-stop
 *   - Node shutdown → all motors stop
 */
class DriverNode : public rclcpp::Node {
public:
    DriverNode() : Node("driver_node") {
        declare_parameters();
        setup_controller();
        setup_ros();

        RCLCPP_INFO(get_logger(),
                    "Driver node started. port=%s baud=%d mode=%s freq=%.1f Hz",
                    port_.c_str(), baudrate_,
                    mode_is_pwm_ ? "pwm" : "spd", control_freq_);
    }

    ~DriverNode() override {
        if (controller_) controller_->stop_all();
        RCLCPP_INFO(get_logger(), "Driver node shut down – motors stopped.");
    }

private:
    // ---- Parameters ----
    void declare_parameters() {
        // Serial
        declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        declare_parameter<int>("baudrate", 115200);

        // Channel mapping
        declare_parameter<int>("channel_fl", 1);
        declare_parameter<int>("channel_fr", 2);
        declare_parameter<int>("channel_rl", 3);
        declare_parameter<int>("channel_rr", 4);
        declare_parameter<bool>("invert_left",  false);
        declare_parameter<bool>("invert_right", false);

        // Control
        declare_parameter<std::string>("control_mode", "spd");   // "spd" or "pwm"
        declare_parameter<int>("pwm_max", 1000);

        // Geometry / limits
        declare_parameter<double>("wheel_radius",      0.033);
        declare_parameter<double>("track_width",       0.23);
        declare_parameter<double>("max_linear_speed",  0.5);
        declare_parameter<double>("max_angular_speed", 2.0);

        // PID (used only in "pwm" mode)
        declare_parameter<double>("pid_kp", 1.0);
        declare_parameter<double>("pid_ki", 0.5);
        declare_parameter<double>("pid_kd", 0.01);
        declare_parameter<double>("pid_max_output",   1.0);
        declare_parameter<double>("pid_max_integral", 0.5);

        // Loop
        declare_parameter<double>("control_frequency", 50.0);
        declare_parameter<double>("cmd_vel_timeout",   0.5);

        // Board-side config (sent once on startup)
        declare_parameter<bool>("apply_board_config",  false);
        declare_parameter<int>("motor_type",      0);
        declare_parameter<int>("deadzone",        0);
        declare_parameter<int>("encoder_lines",   0);
        declare_parameter<int>("motor_phase",     0);
        declare_parameter<int>("wheel_diameter_mm", 0);
        declare_parameter<int>("upload_mall", 0);
        declare_parameter<int>("upload_mtep", 1);
        declare_parameter<int>("upload_mspd", 1);
    }

    void setup_controller() {
        BaseController::Config cfg;
        cfg.port     = get_parameter("serial_port").as_string();
        cfg.baudrate = get_parameter("baudrate").as_int();
        port_        = cfg.port;
        baudrate_    = cfg.baudrate;

        cfg.channels.fl = get_parameter("channel_fl").as_int();
        cfg.channels.fr = get_parameter("channel_fr").as_int();
        cfg.channels.rl = get_parameter("channel_rl").as_int();
        cfg.channels.rr = get_parameter("channel_rr").as_int();
        cfg.invert_left  = get_parameter("invert_left").as_bool();
        cfg.invert_right = get_parameter("invert_right").as_bool();

        std::string mode_str = get_parameter("control_mode").as_string();
        cfg.mode = (mode_str == "pwm") ? BaseController::ControlMode::Pwm
                                       : BaseController::ControlMode::Speed;
        mode_is_pwm_ = (cfg.mode == BaseController::ControlMode::Pwm);
        cfg.pwm_max  = get_parameter("pwm_max").as_int();

        cfg.wheel_radius      = get_parameter("wheel_radius").as_double();
        cfg.track_width       = get_parameter("track_width").as_double();
        cfg.max_linear_speed  = get_parameter("max_linear_speed").as_double();
        cfg.max_angular_speed = get_parameter("max_angular_speed").as_double();

        control_freq_ = get_parameter("control_frequency").as_double();

        controller_ = std::make_unique<BaseController>(cfg);

        controller_->set_pid_gains(
            get_parameter("pid_kp").as_double(),
            get_parameter("pid_ki").as_double(),
            get_parameter("pid_kd").as_double(),
            get_parameter("pid_max_output").as_double(),
            get_parameter("pid_max_integral").as_double());

        if (!controller_->is_initialized()) {
            RCLCPP_ERROR(get_logger(),
                "Failed to open serial port %s @ %d. "
                "Check device node and user permissions (dialout group).",
                port_.c_str(), baudrate_);
            return;
        }

        if (get_parameter("apply_board_config").as_bool()) {
            auto& d = controller_->driver();
            if (int v = get_parameter("motor_type").as_int();       v > 0) d.set_motor_type(v);
            if (int v = get_parameter("deadzone").as_int();         v > 0) d.set_deadzone(v);
            if (int v = get_parameter("encoder_lines").as_int();    v > 0) d.set_encoder_lines(v);
            if (int v = get_parameter("motor_phase").as_int();      v > 0) d.set_motor_phase(v);
            if (int v = get_parameter("wheel_diameter_mm").as_int();v > 0) d.set_wheel_diameter_mm(v);
            d.set_upload(get_parameter("upload_mall").as_int(),
                         get_parameter("upload_mtep").as_int(),
                         get_parameter("upload_mspd").as_int());
            RCLCPP_INFO(get_logger(), "Applied board configuration.");
        }
    }

    void setup_ros() {
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (controller_) controller_->set_target(msg->linear.x, msg->angular.z);
                last_cmd_time_ = now();
            });

        pub_left_target_  = create_publisher<std_msgs::msg::Float64>("driver/left_target", 10);
        pub_right_target_ = create_publisher<std_msgs::msg::Float64>("driver/right_target", 10);
        pub_left_output_  = create_publisher<std_msgs::msg::Float64>("driver/left_output", 10);
        pub_right_output_ = create_publisher<std_msgs::msg::Float64>("driver/right_output", 10);
        pub_speeds_       = create_publisher<std_msgs::msg::Float64MultiArray>("driver/motor_speeds", 10);
        pub_encoders_     = create_publisher<std_msgs::msg::Int32MultiArray>("driver/motor_encoders", 10);

        double period_ms = 1000.0 / control_freq_;
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&DriverNode::control_callback, this));

        last_cmd_time_ = now();
    }

    void control_callback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!controller_) return;

        double timeout_sec = get_parameter("cmd_vel_timeout").as_double();
        if ((now() - last_cmd_time_).seconds() > timeout_sec) {
            controller_->stop_all();
        } else {
            controller_->update(1.0 / control_freq_);
        }

        // Publish debug + feedback
        std_msgs::msg::Float64 d;
        d.data = controller_->target_left_vel();   pub_left_target_->publish(d);
        d.data = controller_->target_right_vel();  pub_right_target_->publish(d);
        d.data = controller_->output_left_duty();  pub_left_output_->publish(d);
        d.data = controller_->output_right_duty(); pub_right_output_->publish(d);

        auto fb = controller_->driver().feedback();
        if (fb.has_speed) {
            std_msgs::msg::Float64MultiArray m;
            m.data.assign(fb.speeds.begin(), fb.speeds.end());
            pub_speeds_->publish(m);
        }
        if (fb.has_encoder) {
            std_msgs::msg::Int32MultiArray m;
            m.data.assign(fb.encoders.begin(), fb.encoders.end());
            pub_encoders_->publish(m);
        }
    }

    // ---- Members ----
    std::unique_ptr<BaseController> controller_;
    std::mutex mutex_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_output_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_output_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_speeds_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_encoders_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Time last_cmd_time_;

    std::string port_;
    int baudrate_{0};
    bool mode_is_pwm_{false};
    double control_freq_{50.0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
