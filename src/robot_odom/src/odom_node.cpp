#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "robot_odom/encoder.hpp"
#include "robot_odom/odometry.hpp"

#include <memory>
#include <cmath>
#include <vector>
#include <array>

/**
 * Odometry node for 4-wheel skid-steer robot.
 *
 * Reads four quadrature (or single-phase) encoders, computes per-side
 * average velocity, integrates differential-drive odometry, and publishes:
 *   /wheel/odometry   (nav_msgs/Odometry)
 *   /wheel/left_vel   (std_msgs/Float64) – for PID feedback in driver_node
 *   /wheel/right_vel  (std_msgs/Float64) – for PID feedback in driver_node
 *   /joint_states      (sensor_msgs/JointState) – for visualization
 *
 * Does NOT publish odom → base_link TF (EKF handles that).
 */
class OdomNode : public rclcpp::Node {
public:
    OdomNode()
        : Node("odom_node") {
        declare_params();
        init_encoders();
        init_odometry();
        init_ros();

        RCLCPP_INFO(get_logger(),
            "Odom node started. single_phase=%s, freq=%.1f Hz, "
            "ticks/wheel_rev=%.1f",
            single_phase_ ? "true" : "false",
            odom_freq_, encoders_[0]->ticks_per_wheel_rev());
    }

    ~OdomNode() override {
        RCLCPP_INFO(get_logger(), "Odom node shutting down.");
    }

private:
    void declare_params() {
        declare_parameter<std::string>("gpio_chip", "/dev/gpiochip4");
        declare_parameter<int>("encoder_fl_a", 2);
        declare_parameter<int>("encoder_fl_b", 3);
        declare_parameter<int>("encoder_fr_a", 4);
        declare_parameter<int>("encoder_fr_b", 14);
        declare_parameter<int>("encoder_rl_a", 15);
        declare_parameter<int>("encoder_rl_b", 18);
        declare_parameter<int>("encoder_rr_a", 7);
        declare_parameter<int>("encoder_rr_b", 8);

        declare_parameter<bool>("single_phase", false);
        declare_parameter<int>("counts_per_rev", 11);
        declare_parameter<double>("gear_ratio", 30.0);
        declare_parameter<double>("wheel_radius", 0.033);
        declare_parameter<double>("track_width", 0.23);
        declare_parameter<double>("odom_frequency", 50.0);
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::string>("base_frame", "base_link");

        declare_parameter<std::vector<double>>("pose_covariance_diagonal",
            {0.001, 0.001, 0.0, 0.0, 0.0, 0.01});
        declare_parameter<std::vector<double>>("twist_covariance_diagonal",
            {0.001, 0.001, 0.0, 0.0, 0.0, 0.01});
    }

    void init_encoders() {
        auto chip = get_parameter("gpio_chip").as_string();
        single_phase_ = get_parameter("single_phase").as_bool();
        int cpr = get_parameter("counts_per_rev").as_int();
        double gr = get_parameter("gear_ratio").as_double();
        wheel_radius_ = get_parameter("wheel_radius").as_double();

        // Pin arrays: [FL, FR, RL, RR]
        int pins_a[4] = {
            static_cast<int>(get_parameter("encoder_fl_a").as_int()),
            static_cast<int>(get_parameter("encoder_fr_a").as_int()),
            static_cast<int>(get_parameter("encoder_rl_a").as_int()),
            static_cast<int>(get_parameter("encoder_rr_a").as_int()),
        };
        int pins_b[4] = {
            static_cast<int>(get_parameter("encoder_fl_b").as_int()),
            static_cast<int>(get_parameter("encoder_fr_b").as_int()),
            static_cast<int>(get_parameter("encoder_rl_b").as_int()),
            static_cast<int>(get_parameter("encoder_rr_b").as_int()),
        };

        const char* names[4] = {"FL", "FR", "RL", "RR"};
        for (int i = 0; i < 4; i++) {
            encoders_[i] = std::make_unique<Encoder>(
                chip, pins_a[i], pins_b[i], single_phase_, cpr, gr);
            if (!encoders_[i]->is_initialized()) {
                RCLCPP_ERROR(get_logger(),
                    "Encoder %s failed to init (A=%d B=%d). Check wiring/permissions.",
                    names[i], pins_a[i], pins_b[i]);
            }
        }
    }

    void init_odometry() {
        double tw = get_parameter("track_width").as_double();
        odom_ = std::make_unique<DiffDriveOdometry>(tw);
        odom_freq_ = get_parameter("odom_frequency").as_double();

        for (auto& t : prev_ticks_) t = 0;
    }

    void init_ros() {
        odom_pub_       = create_publisher<nav_msgs::msg::Odometry>("wheel/odometry", 10);
        left_vel_pub_   = create_publisher<std_msgs::msg::Float64>("wheel/left_vel", 10);
        right_vel_pub_  = create_publisher<std_msgs::msg::Float64>("wheel/right_vel", 10);
        joint_pub_      = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        double period_ms = 1000.0 / odom_freq_;
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&OdomNode::timer_callback, this));
    }

    void timer_callback() {
        auto now_stamp = now();
        double dt = 1.0 / odom_freq_;

        // Read encoder ticks
        int64_t cur_ticks[4];
        for (int i = 0; i < 4; i++) {
            cur_ticks[i] = encoders_[i]->get_ticks();
        }

        // Delta ticks
        int64_t delta[4];
        for (int i = 0; i < 4; i++) {
            delta[i] = cur_ticks[i] - prev_ticks_[i];
            prev_ticks_[i] = cur_ticks[i];
        }

        // Convert delta ticks to wheel linear velocity
        double tpr = encoders_[0]->ticks_per_wheel_rev();
        double vel[4];
        for (int i = 0; i < 4; i++) {
            // radians turned by the wheel
            double rad = (static_cast<double>(delta[i]) / tpr) * 2.0 * M_PI;
            vel[i] = (rad * wheel_radius_) / dt;
        }

        // Average per side: FL=0, FR=1, RL=2, RR=3
        double left_vel  = (vel[0] + vel[2]) / 2.0;
        double right_vel = (vel[1] + vel[3]) / 2.0;

        // Update odometry
        odom_->update(left_vel, right_vel, dt);

        // Accumulate joint positions (for visualization)
        for (int i = 0; i < 4; i++) {
            double drad = (static_cast<double>(delta[i]) / tpr) * 2.0 * M_PI;
            joint_positions_[i] += drad;
        }

        // ---- Publish wheel/odometry ----
        auto odom_msg = nav_msgs::msg::Odometry();
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

        // Fill covariance
        auto pose_cov = get_parameter("pose_covariance_diagonal").as_double_array();
        auto twist_cov = get_parameter("twist_covariance_diagonal").as_double_array();
        for (int i = 0; i < 6 && i < static_cast<int>(pose_cov.size()); i++) {
            odom_msg.pose.covariance[i * 7] = pose_cov[i];
        }
        for (int i = 0; i < 6 && i < static_cast<int>(twist_cov.size()); i++) {
            odom_msg.twist.covariance[i * 7] = twist_cov[i];
        }

        odom_pub_->publish(odom_msg);

        // ---- Publish left/right velocity for PID ----
        auto lv = std_msgs::msg::Float64();
        lv.data = left_vel;
        left_vel_pub_->publish(lv);

        auto rv = std_msgs::msg::Float64();
        rv.data = right_vel;
        right_vel_pub_->publish(rv);

        // ---- Publish joint_states ----
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = now_stamp;
        js.name = {"front_left_wheel_joint", "front_right_wheel_joint",
                    "rear_left_wheel_joint", "rear_right_wheel_joint"};
        js.position = {joint_positions_[0], joint_positions_[1],
                       joint_positions_[2], joint_positions_[3]};
        js.velocity = {vel[0] / wheel_radius_, vel[1] / wheel_radius_,
                       vel[2] / wheel_radius_, vel[3] / wheel_radius_};
        joint_pub_->publish(js);
    }

    // Encoders: FL=0, FR=1, RL=2, RR=3
    std::array<std::unique_ptr<Encoder>, 4> encoders_;
    std::unique_ptr<DiffDriveOdometry> odom_;
    int64_t prev_ticks_[4]{0, 0, 0, 0};
    double joint_positions_[4]{0.0, 0.0, 0.0, 0.0};
    double wheel_radius_{0.033};
    double odom_freq_{50.0};
    bool single_phase_{false};

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
