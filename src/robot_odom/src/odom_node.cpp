#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "robot_odom/odometry.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <mutex>

/**
 * Odometry node for 4-wheel skid-steer robot.
 *
 * Encoder ticks come from the motor-driver board via serial (robot_driver),
 * published on /driver/motor_encoders (Int32MultiArray, 4 channels).
 *
 * Publishes:
 *   /wheel/odometry   (nav_msgs/Odometry)
 *   /wheel/left_vel   (std_msgs/Float64) – for PID feedback
 *   /wheel/right_vel  (std_msgs/Float64) – for PID feedback
 *   /joint_states     (sensor_msgs/JointState)
 *
 * Does NOT publish odom → base_link TF (EKF handles that).
 */
class OdomNode : public rclcpp::Node {
public:
    OdomNode() : Node("odom_node") {
        declare_params();
        init_odometry();
        init_ros();

        RCLCPP_INFO(get_logger(),
            "Odom node started. freq=%.1f Hz, ticks/wheel_rev=%.1f, "
            "channels [FL=%d FR=%d RL=%d RR=%d]",
            odom_freq_, ticks_per_wheel_rev_,
            ch_[0], ch_[1], ch_[2], ch_[3]);
    }

    ~OdomNode() override {
        RCLCPP_INFO(get_logger(), "Odom node shutting down.");
    }

private:
    void declare_params() {
        // Source topic for encoder ticks (from robot_driver)
        declare_parameter<std::string>("encoder_topic", "/driver/motor_encoders");

        // Channel mapping (must match driver_params.yaml channel_{fl,fr,rl,rr})
        // 1-based on the board; array index = channel - 1.
        declare_parameter<int>("channel_fl", 1);
        declare_parameter<int>("channel_fr", 2);
        declare_parameter<int>("channel_rl", 3);
        declare_parameter<int>("channel_rr", 4);

        // Sign inversion if a wheel's encoder counts opposite to its motion
        declare_parameter<bool>("invert_fl", false);
        declare_parameter<bool>("invert_fr", false);
        declare_parameter<bool>("invert_rl", false);
        declare_parameter<bool>("invert_rr", false);

        // Encoder characteristics
        declare_parameter<int>("counts_per_rev", 11);    // PPR on motor shaft
        declare_parameter<double>("gear_ratio", 30.0);   // motor revs per wheel rev
        declare_parameter<bool>("quadrature", true);     // board decodes x4 edges

        // Robot geometry
        declare_parameter<double>("wheel_radius", 0.033);
        declare_parameter<double>("track_width", 0.23);

        // Loop
        declare_parameter<double>("odom_frequency", 50.0);

        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "base_link");

        declare_parameter<std::vector<double>>("pose_covariance_diagonal",
            {0.001, 0.001, 0.0, 0.0, 0.0, 0.01});
        declare_parameter<std::vector<double>>("twist_covariance_diagonal",
            {0.001, 0.001, 0.0, 0.0, 0.0, 0.01});
    }

    void init_odometry() {
        double tw = get_parameter("track_width").as_double();
        odom_ = std::make_unique<DiffDriveOdometry>(tw);
        odom_freq_    = get_parameter("odom_frequency").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();

        int cpr    = get_parameter("counts_per_rev").as_int();
        double gr  = get_parameter("gear_ratio").as_double();
        bool quad  = get_parameter("quadrature").as_bool();
        double edges_per_pulse = quad ? 4.0 : 1.0;
        ticks_per_wheel_rev_ = static_cast<double>(cpr) * edges_per_pulse * gr;

        // Channel mapping FL, FR, RL, RR → array indices on the incoming msg
        ch_[0] = get_parameter("channel_fl").as_int() - 1;
        ch_[1] = get_parameter("channel_fr").as_int() - 1;
        ch_[2] = get_parameter("channel_rl").as_int() - 1;
        ch_[3] = get_parameter("channel_rr").as_int() - 1;

        sign_[0] = get_parameter("invert_fl").as_bool() ? -1 : 1;
        sign_[1] = get_parameter("invert_fr").as_bool() ? -1 : 1;
        sign_[2] = get_parameter("invert_rl").as_bool() ? -1 : 1;
        sign_[3] = get_parameter("invert_rr").as_bool() ? -1 : 1;

        for (auto& t : prev_ticks_) t = 0;
        for (auto& p : joint_positions_) p = 0.0;
    }

    void init_ros() {
        auto topic = get_parameter("encoder_topic").as_string();
        enc_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            topic, rclcpp::SensorDataQoS(),
            std::bind(&OdomNode::encoder_callback, this, std::placeholders::_1));

        odom_pub_      = create_publisher<nav_msgs::msg::Odometry>("wheel/odometry", 10);
        left_vel_pub_  = create_publisher<std_msgs::msg::Float64>("wheel/left_vel", 10);
        right_vel_pub_ = create_publisher<std_msgs::msg::Float64>("wheel/right_vel", 10);
        joint_pub_     = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        double period_ms = 1000.0 / odom_freq_;
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&OdomNode::timer_callback, this));
    }

    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "encoder msg has only %zu elements (expected >=4)",
                msg->data.size());
            return;
        }
        std::lock_guard<std::mutex> lock(tick_mtx_);
        for (int i = 0; i < 4; ++i) {
            int idx = ch_[i];
            if (idx < 0 || idx >= static_cast<int>(msg->data.size())) continue;
            latest_ticks_[i] = static_cast<int64_t>(msg->data[idx]) * sign_[i];
        }
        have_ticks_ = true;
    }

    void timer_callback() {
        auto now_stamp = now();
        double dt = 1.0 / odom_freq_;

        int64_t cur_ticks[4];
        bool have;
        {
            std::lock_guard<std::mutex> lock(tick_mtx_);
            have = have_ticks_;
            for (int i = 0; i < 4; ++i) cur_ticks[i] = latest_ticks_[i];
        }

        if (!have) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "no encoder data yet on '%s'",
                get_parameter("encoder_topic").as_string().c_str());
            return;
        }

        // First valid sample: seed prev_ticks, skip integration this cycle.
        if (!prev_seeded_) {
            for (int i = 0; i < 4; ++i) prev_ticks_[i] = cur_ticks[i];
            prev_seeded_ = true;
            return;
        }

        int64_t delta[4];
        for (int i = 0; i < 4; ++i) {
            delta[i] = cur_ticks[i] - prev_ticks_[i];
            prev_ticks_[i] = cur_ticks[i];
        }

        double vel[4];
        for (int i = 0; i < 4; ++i) {
            double rad = (static_cast<double>(delta[i]) / ticks_per_wheel_rev_) * 2.0 * M_PI;
            vel[i] = (rad * wheel_radius_) / dt;
            joint_positions_[i] += rad;
        }

        // FL=0, FR=1, RL=2, RR=3
        double left_vel  = (vel[0] + vel[2]) / 2.0;
        double right_vel = (vel[1] + vel[3]) / 2.0;

        odom_->update(left_vel, right_vel, dt);

        // ---- wheel/odometry ----
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now_stamp;
        odom_msg.header.frame_id = get_parameter("odom_frame").as_string();
        odom_msg.child_frame_id  = get_parameter("base_frame").as_string();

        odom_msg.pose.pose.position.x = odom_->x();
        odom_msg.pose.pose.position.y = odom_->y();
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, odom_->theta());
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x  = odom_->linear_vel();
        odom_msg.twist.twist.angular.z = odom_->angular_vel();

        auto pose_cov  = get_parameter("pose_covariance_diagonal").as_double_array();
        auto twist_cov = get_parameter("twist_covariance_diagonal").as_double_array();
        for (int i = 0; i < 6 && i < static_cast<int>(pose_cov.size()); i++) {
            odom_msg.pose.covariance[i * 7] = pose_cov[i];
        }
        for (int i = 0; i < 6 && i < static_cast<int>(twist_cov.size()); i++) {
            odom_msg.twist.covariance[i * 7] = twist_cov[i];
        }
        odom_pub_->publish(odom_msg);

        // ---- per-side velocity (PID feedback) ----
        std_msgs::msg::Float64 lv; lv.data = left_vel;  left_vel_pub_->publish(lv);
        std_msgs::msg::Float64 rv; rv.data = right_vel; right_vel_pub_->publish(rv);

        // ---- joint_states ----
        sensor_msgs::msg::JointState js;
        js.header.stamp = now_stamp;
        js.name = {"front_left_wheel_joint", "front_right_wheel_joint",
                   "rear_left_wheel_joint",  "rear_right_wheel_joint"};
        js.position = {joint_positions_[0], joint_positions_[1],
                       joint_positions_[2], joint_positions_[3]};
        js.velocity = {vel[0] / wheel_radius_, vel[1] / wheel_radius_,
                       vel[2] / wheel_radius_, vel[3] / wheel_radius_};
        joint_pub_->publish(js);
    }

    // --- state ---
    std::unique_ptr<DiffDriveOdometry> odom_;

    std::mutex tick_mtx_;
    int64_t latest_ticks_[4]{0, 0, 0, 0};
    bool have_ticks_{false};

    int64_t prev_ticks_[4]{0, 0, 0, 0};
    bool prev_seeded_{false};

    double joint_positions_[4]{0.0, 0.0, 0.0, 0.0};

    int ch_[4]{0, 1, 2, 3};
    int sign_[4]{1, 1, 1, 1};

    double ticks_per_wheel_rev_{1320.0};
    double wheel_radius_{0.033};
    double odom_freq_{50.0};

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr enc_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
