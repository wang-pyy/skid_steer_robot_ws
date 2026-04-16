#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

#include "robot_bno055/bno055_driver.hpp"

#include <memory>
#include <chrono>
#include <vector>

/**
 * ROS 2 node for BNO055 IMU.
 *
 * Publishes:
 *   /imu/data   (sensor_msgs/Imu)           – orientation, angular velocity, linear acceleration
 *   /imu/mag    (sensor_msgs/MagneticField)  – magnetometer
 *   /imu/temp   (std_msgs/Float64)           – temperature in °C
 *
 * The BNO055 is used in NDOF mode (default), reading the chip's own
 * fused quaternion. This is preferred over external fusion because:
 *   1. The BNO055 has a dedicated ARM Cortex-M0+ running Bosch's
 *      proprietary sensor fusion at 100 Hz internally.
 *   2. It handles gyro drift correction and gravity subtraction.
 *   3. It saves CPU on the Raspberry Pi.
 *
 * MAGNETIC INTERFERENCE NOTE:
 *   Indoor environments with motors and metal may degrade heading accuracy.
 *   Monitor the calibration status – if mag calibration stays at 0,
 *   consider switching to IMU mode (operation_mode: 8) which disables
 *   magnetometer fusion. Heading will drift but roll/pitch remain accurate.
 */
class BNO055Node : public rclcpp::Node {
public:
    BNO055Node()
        : Node("bno055_node") {
        declare_params();
        init_driver();
        init_ros();
    }

    ~BNO055Node() override {
        RCLCPP_INFO(get_logger(), "BNO055 node shutting down.");
    }

private:
    void declare_params() {
        declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
        declare_parameter<int>("i2c_address", 0x28);
        declare_parameter<std::string>("frame_id", "imu_link");
        declare_parameter<double>("publish_frequency", 50.0);
        declare_parameter<int>("operation_mode", 0x0C);  // NDOF

        // Covariance values (3 diagonal elements each)
        // BNO055 heading accuracy: ~3° → (3*pi/180)^2 ≈ 0.0027
        // Gyro noise: ~0.02 rad/s → 0.0004
        // Accel noise: ~0.08 m/s² → 0.0064
        declare_parameter<std::vector<double>>("orientation_covariance",
            {0.0025, 0.0025, 0.0025});
        declare_parameter<std::vector<double>>("angular_velocity_covariance",
            {0.0004, 0.0004, 0.0004});
        declare_parameter<std::vector<double>>("linear_acceleration_covariance",
            {0.0064, 0.0064, 0.0064});
    }

    void init_driver() {
        auto bus = get_parameter("i2c_bus").as_string();
        auto addr = static_cast<uint8_t>(get_parameter("i2c_address").as_int());
        auto mode = static_cast<uint8_t>(get_parameter("operation_mode").as_int());

        driver_ = std::make_unique<BNO055Driver>(bus, addr);

        // Retry initialization up to 5 times
        for (int attempt = 1; attempt <= 5; attempt++) {
            RCLCPP_INFO(get_logger(),
                "Initializing BNO055 (attempt %d/5) on %s addr=0x%02X mode=0x%02X",
                attempt, bus.c_str(), addr, mode);

            if (driver_->initialize(mode)) {
                RCLCPP_INFO(get_logger(), "BNO055 initialized successfully.");
                return;
            }

            RCLCPP_WARN(get_logger(),
                "BNO055 init failed (attempt %d). Retrying in 2 seconds...", attempt);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        RCLCPP_ERROR(get_logger(),
            "BNO055 initialization failed after 5 attempts. "
            "Check I2C bus, address, and wiring. "
            "Ensure I2C is enabled (raspi-config) and the device is visible (i2cdetect -y 1). "
            "Node will continue but publish empty data.");
    }

    void init_ros() {
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        mag_pub_  = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
        temp_pub_ = create_publisher<std_msgs::msg::Float64>("imu/temp", 10);

        double freq = get_parameter("publish_frequency").as_double();
        double period_ms = 1000.0 / freq;
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&BNO055Node::publish_callback, this));

        // Calibration check at 0.2 Hz
        calib_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&BNO055Node::calib_callback, this));
    }

    void publish_callback() {
        if (!driver_ || !driver_->is_initialized()) return;

        auto stamp = now();
        auto frame = get_parameter("frame_id").as_string();

        auto ori_cov = get_parameter("orientation_covariance").as_double_array();
        auto gyro_cov = get_parameter("angular_velocity_covariance").as_double_array();
        auto acc_cov = get_parameter("linear_acceleration_covariance").as_double_array();

        // ---- IMU message ----
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = frame;

        // Orientation (quaternion from BNO055 fusion)
        double qw, qx, qy, qz;
        bool quat_ok = driver_->read_quaternion(qw, qx, qy, qz);
        if (quat_ok) {
            imu_msg.orientation.w = qw;
            imu_msg.orientation.x = qx;
            imu_msg.orientation.y = qy;
            imu_msg.orientation.z = qz;
            // Set covariance diagonal
            imu_msg.orientation_covariance[0] = ori_cov[0];
            imu_msg.orientation_covariance[4] = ori_cov[1];
            imu_msg.orientation_covariance[8] = ori_cov[2];
        } else {
            // Orientation unknown – set covariance[0] = -1 per REP-145
            imu_msg.orientation_covariance[0] = -1.0;
        }

        // Angular velocity
        double gx, gy, gz;
        if (driver_->read_gyroscope(gx, gy, gz)) {
            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;
            imu_msg.angular_velocity_covariance[0] = gyro_cov[0];
            imu_msg.angular_velocity_covariance[4] = gyro_cov[1];
            imu_msg.angular_velocity_covariance[8] = gyro_cov[2];
        }

        // Linear acceleration (gravity removed)
        double ax, ay, az;
        bool lia_ok = driver_->read_linear_acceleration(ax, ay, az);
        if (!lia_ok) {
            // Fall back to raw accelerometer
            lia_ok = driver_->read_accelerometer(ax, ay, az);
        }
        if (lia_ok) {
            imu_msg.linear_acceleration.x = ax;
            imu_msg.linear_acceleration.y = ay;
            imu_msg.linear_acceleration.z = az;
            imu_msg.linear_acceleration_covariance[0] = acc_cov[0];
            imu_msg.linear_acceleration_covariance[4] = acc_cov[1];
            imu_msg.linear_acceleration_covariance[8] = acc_cov[2];
        }

        imu_pub_->publish(imu_msg);

        // ---- Magnetometer ----
        double mx, my, mz;
        if (driver_->read_magnetometer(mx, my, mz)) {
            auto mag_msg = sensor_msgs::msg::MagneticField();
            mag_msg.header.stamp = stamp;
            mag_msg.header.frame_id = frame;
            // Convert µT to Tesla (SI unit for MagneticField message)
            mag_msg.magnetic_field.x = mx * 1e-6;
            mag_msg.magnetic_field.y = my * 1e-6;
            mag_msg.magnetic_field.z = mz * 1e-6;
            mag_pub_->publish(mag_msg);
        }

        // ---- Temperature ----
        int8_t temp;
        if (driver_->read_temperature(temp)) {
            auto temp_msg = std_msgs::msg::Float64();
            temp_msg.data = static_cast<double>(temp);
            temp_pub_->publish(temp_msg);
        }
    }

    void calib_callback() {
        if (!driver_ || !driver_->is_initialized()) return;

        uint8_t sys, gyro, accel, mag;
        if (driver_->read_calibration(sys, gyro, accel, mag)) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                "BNO055 calibration – sys:%d gyro:%d accel:%d mag:%d",
                sys, gyro, accel, mag);

            if (mag < 2) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
                    "Magnetometer calibration poor (mag=%d). "
                    "Heading may be inaccurate. "
                    "Try moving the robot in a figure-8 pattern, "
                    "or switch to IMU mode (operation_mode: 8) to disable mag.",
                    mag);
            }
        }
    }

    std::unique_ptr<BNO055Driver> driver_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr calib_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BNO055Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
