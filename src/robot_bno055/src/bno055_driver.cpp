#include "robot_bno055/bno055_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <cmath>

BNO055Driver::BNO055Driver(const std::string& i2c_bus, uint8_t address)
    : address_(address) {
    fd_ = open(i2c_bus.c_str(), O_RDWR);
    if (fd_ < 0) {
        return;
    }
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        close(fd_);
        fd_ = -1;
    }
}

BNO055Driver::~BNO055Driver() {
    if (fd_ >= 0) {
        // Return to config mode before closing
        write_byte(REG_OPR_MODE, MODE_CONFIG);
        close(fd_);
    }
}

bool BNO055Driver::initialize(uint8_t mode) {
    if (fd_ < 0) return false;

    // 1. Verify chip ID
    uint8_t chip_id = 0;
    if (!read_bytes(REG_CHIP_ID, &chip_id, 1) || chip_id != BNO055_CHIP_ID) {
        return false;
    }

    // 2. Switch to CONFIG mode
    if (!write_byte(REG_OPR_MODE, MODE_CONFIG)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // 3. Reset the chip
    if (!write_byte(REG_SYS_TRIGGER, 0x20)) return false;
    // Wait for reset to complete (BNO055 takes ~650 ms after reset)
    std::this_thread::sleep_for(std::chrono::milliseconds(700));

    // 4. Re-verify chip ID after reset
    chip_id = 0;
    for (int attempt = 0; attempt < 10; attempt++) {
        if (read_bytes(REG_CHIP_ID, &chip_id, 1) && chip_id == BNO055_CHIP_ID) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (chip_id != BNO055_CHIP_ID) {
        return false;
    }

    // 5. Set power mode to NORMAL
    if (!write_byte(REG_PWR_MODE, 0x00)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 6. Set unit selection:
    //    Acceleration: m/s², Angular rate: rad/s, Euler: radians,
    //    Temperature: Celsius, Orientation: Android (default)
    //    Bit layout: [7:acc_unit(0=m/s²)] [4:euler_unit(1=rad)] [2:gyro_unit(1=rps)] ...
    //    We want: accel=m/s²(0), gyro=rad/s(bit1=1), euler=rad(bit2=1)  → 0x06
    //    Actually: bit 0 = ACC (0=m/s²), bit 1 = GYR (0=dps,1=rps), bit 2 = EUL (0=deg,1=rad)
    //    We want gyro in rps and angles in radians: 0b00000110 = 0x06
    //    But for this driver we convert manually, so we can leave defaults.
    //    Let's set gyro to DPS (default) and convert ourselves for maximum compatibility.
    //    Default UNIT_SEL = 0x00 → Accel: m/s², Gyro: DPS, Euler: degrees
    //    We'll convert gyro from DPS to rad/s in read_gyroscope().

    // 7. Set operating mode
    if (!write_byte(REG_OPR_MODE, mode)) return false;
    // Mode switch takes ~7ms for non-config, ~19ms for config→any
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    initialized_ = true;
    return true;
}

bool BNO055Driver::read_quaternion(double& w, double& x, double& y, double& z) {
    uint8_t buf[8];
    if (!read_bytes(REG_QUA_DATA_W_LSB, buf, 8)) return false;

    // 1 quaternion unit = 2^14 = 16384 LSB
    constexpr double QUAT_SCALE = 1.0 / 16384.0;

    int16_t raw_w = combine_16le(buf[0], buf[1]);
    int16_t raw_x = combine_16le(buf[2], buf[3]);
    int16_t raw_y = combine_16le(buf[4], buf[5]);
    int16_t raw_z = combine_16le(buf[6], buf[7]);

    w = raw_w * QUAT_SCALE;
    x = raw_x * QUAT_SCALE;
    y = raw_y * QUAT_SCALE;
    z = raw_z * QUAT_SCALE;

    return true;
}

bool BNO055Driver::read_gyroscope(double& gx, double& gy, double& gz) {
    uint8_t buf[6];
    if (!read_bytes(REG_GYR_DATA_X_LSB, buf, 6)) return false;

    // Default unit: DPS (degrees per second), 1 LSB = 1/16 DPS
    // Convert to rad/s: value/16.0 * pi/180
    constexpr double GYRO_SCALE = (1.0 / 16.0) * M_PI / 180.0;

    gx = combine_16le(buf[0], buf[1]) * GYRO_SCALE;
    gy = combine_16le(buf[2], buf[3]) * GYRO_SCALE;
    gz = combine_16le(buf[4], buf[5]) * GYRO_SCALE;

    return true;
}

bool BNO055Driver::read_linear_acceleration(double& ax, double& ay, double& az) {
    uint8_t buf[6];
    if (!read_bytes(REG_LIA_DATA_X_LSB, buf, 6)) return false;

    // 1 LSB = 0.01 m/s²
    constexpr double LIA_SCALE = 0.01;

    ax = combine_16le(buf[0], buf[1]) * LIA_SCALE;
    ay = combine_16le(buf[2], buf[3]) * LIA_SCALE;
    az = combine_16le(buf[4], buf[5]) * LIA_SCALE;

    return true;
}

bool BNO055Driver::read_accelerometer(double& ax, double& ay, double& az) {
    uint8_t buf[6];
    if (!read_bytes(REG_ACC_DATA_X_LSB, buf, 6)) return false;

    // 1 LSB = 0.01 m/s²
    constexpr double ACC_SCALE = 0.01;

    ax = combine_16le(buf[0], buf[1]) * ACC_SCALE;
    ay = combine_16le(buf[2], buf[3]) * ACC_SCALE;
    az = combine_16le(buf[4], buf[5]) * ACC_SCALE;

    return true;
}

bool BNO055Driver::read_magnetometer(double& mx, double& my, double& mz) {
    uint8_t buf[6];
    if (!read_bytes(REG_MAG_DATA_X_LSB, buf, 6)) return false;

    // 1 LSB = 1/16 µT
    constexpr double MAG_SCALE = 1.0 / 16.0;

    mx = combine_16le(buf[0], buf[1]) * MAG_SCALE;
    my = combine_16le(buf[2], buf[3]) * MAG_SCALE;
    mz = combine_16le(buf[4], buf[5]) * MAG_SCALE;

    return true;
}

bool BNO055Driver::read_temperature(int8_t& temp) {
    uint8_t buf;
    if (!read_bytes(REG_TEMP, &buf, 1)) return false;
    temp = static_cast<int8_t>(buf);
    return true;
}

bool BNO055Driver::read_calibration(uint8_t& sys, uint8_t& gyro,
                                     uint8_t& accel, uint8_t& mag) {
    uint8_t buf;
    if (!read_bytes(REG_CALIB_STAT, &buf, 1)) return false;

    sys   = (buf >> 6) & 0x03;
    gyro  = (buf >> 4) & 0x03;
    accel = (buf >> 2) & 0x03;
    mag   =  buf       & 0x03;

    return true;
}

// ---- Private helpers ----

bool BNO055Driver::write_byte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return (write(fd_, buf, 2) == 2);
}

bool BNO055Driver::read_bytes(uint8_t reg, uint8_t* buf, size_t len) {
    // Write register address, then read data
    if (write(fd_, &reg, 1) != 1) return false;
    if (read(fd_, buf, len) != static_cast<ssize_t>(len)) return false;
    return true;
}

int16_t BNO055Driver::combine_16le(uint8_t lsb, uint8_t msb) {
    return static_cast<int16_t>(static_cast<uint16_t>(msb) << 8 | lsb);
}
