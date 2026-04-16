#pragma once

#include <cstdint>
#include <string>

/**
 * Low-level BNO055 driver via Linux i2c-dev (/dev/i2c-N).
 *
 * Uses NDOF mode by default: the BNO055's on-chip ARM Cortex-M0+
 * runs 9-axis absolute orientation fusion (accelerometer + gyroscope +
 * magnetometer). Reading the fused quaternion from the chip is more
 * accurate than running external sensor fusion on raw data, and saves
 * CPU on the Raspberry Pi.
 *
 * WARNING: In environments with significant magnetic interference
 * (near motors, metal structures, etc.), magnetometer calibration
 * may be poor. In that case, switch to IMU mode (0x08) which uses
 * only accelerometer + gyroscope fusion (no absolute heading).
 */
class BNO055Driver {
public:
    // BNO055 register addresses (Page 0)
    static constexpr uint8_t REG_CHIP_ID          = 0x00;
    static constexpr uint8_t REG_ACC_DATA_X_LSB   = 0x08;
    static constexpr uint8_t REG_MAG_DATA_X_LSB   = 0x0E;
    static constexpr uint8_t REG_GYR_DATA_X_LSB   = 0x14;
    static constexpr uint8_t REG_QUA_DATA_W_LSB   = 0x20;
    static constexpr uint8_t REG_LIA_DATA_X_LSB   = 0x28;
    static constexpr uint8_t REG_TEMP             = 0x34;
    static constexpr uint8_t REG_CALIB_STAT       = 0x35;
    static constexpr uint8_t REG_OPR_MODE         = 0x3D;
    static constexpr uint8_t REG_PWR_MODE         = 0x3E;
    static constexpr uint8_t REG_SYS_TRIGGER      = 0x3F;
    static constexpr uint8_t REG_UNIT_SEL         = 0x3B;

    // Operating modes
    static constexpr uint8_t MODE_CONFIG = 0x00;
    static constexpr uint8_t MODE_IMU    = 0x08;  // Accel + Gyro fusion
    static constexpr uint8_t MODE_NDOF   = 0x0C;  // Full 9-DOF fusion

    // Expected chip ID
    static constexpr uint8_t BNO055_CHIP_ID = 0xA0;

    /**
     * @param i2c_bus   e.g. "/dev/i2c-1"
     * @param address   I2C slave address (default 0x28, alt 0x29)
     */
    BNO055Driver(const std::string& i2c_bus, uint8_t address = 0x28);
    ~BNO055Driver();

    BNO055Driver(const BNO055Driver&) = delete;
    BNO055Driver& operator=(const BNO055Driver&) = delete;

    /**
     * Initialize the sensor: verify chip ID, reset, configure mode.
     * @param mode  Operating mode (MODE_NDOF or MODE_IMU)
     * @return true on success
     */
    bool initialize(uint8_t mode = MODE_NDOF);

    bool is_initialized() const { return initialized_; }

    /** Read fused quaternion (unitless, normalized). */
    bool read_quaternion(double& w, double& x, double& y, double& z);

    /** Read gyroscope angular velocity in rad/s. */
    bool read_gyroscope(double& gx, double& gy, double& gz);

    /** Read linear acceleration (gravity removed) in m/s². */
    bool read_linear_acceleration(double& ax, double& ay, double& az);

    /** Read raw accelerometer in m/s². */
    bool read_accelerometer(double& ax, double& ay, double& az);

    /** Read magnetometer in µT. */
    bool read_magnetometer(double& mx, double& my, double& mz);

    /** Read temperature in °C. */
    bool read_temperature(int8_t& temp);

    /**
     * Read calibration status (0 = uncalibrated, 3 = fully calibrated).
     */
    bool read_calibration(uint8_t& sys, uint8_t& gyro,
                          uint8_t& accel, uint8_t& mag);

private:
    bool write_byte(uint8_t reg, uint8_t value);
    bool read_bytes(uint8_t reg, uint8_t* buf, size_t len);
    int16_t combine_16le(uint8_t lsb, uint8_t msb);

    int fd_{-1};
    uint8_t address_;
    bool initialized_{false};
};
