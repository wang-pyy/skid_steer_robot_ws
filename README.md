# Skid-Steer Robot ROS 2 Workspace

A complete ROS 2 Jazzy workspace for a 4-wheel skid-steer indoor robot,
designed to run on Raspberry Pi 5 inside a Docker container.

## Hardware

| Component    | Model                            | Interface          |
|-------------|----------------------------------|--------------------|
| Computer    | Raspberry Pi 5 (8 GB)            | —                  |
| Motors      | 4× DC gearmotor with encoder    | GPIO (libgpiod)    |
| Motor driver| 2× L298N (or similar H-bridge)  | GPIO               |
| IMU         | BNO055                           | I2C (`/dev/i2c-1`) |
| Lidar       | RPLIDAR A1                       | Serial (`/dev/ttyUSB0`) |

## Packages

| Package            | Description                                        |
|--------------------|----------------------------------------------------|
| `robot_description`| URDF/xacro, robot_state_publisher                  |
| `robot_driver`     | cmd_vel → PWM motor control with PID               |
| `robot_odom`       | Encoder reading → wheel odometry                   |
| `robot_bno055`     | BNO055 IMU driver (I2C)                            |
| `robot_bringup`    | Launch files and config (EKF, SLAM, Nav2)          |

## TF Tree

```
map → odom → base_link → imu_link
                       → laser
                       → front_left_wheel
                       → front_right_wheel
                       → rear_left_wheel
                       → rear_right_wheel
```

- `odom → base_link` is published by EKF (`robot_localization`)
- `map → odom` is published by `slam_toolbox` (SLAM mode) or `amcl` (Nav mode)

## Quick Start

### 1. Install Dependencies (inside Docker container)

```bash
apt-get update && apt-get install -y \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rplidar-ros \
  ros-jazzy-tf2-ros \
  libgpiod-dev \
  i2c-tools
```

### 2. Build

```bash
cd /path/to/skid_steer_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch – Base Only (motors + encoders + IMU + EKF)

```bash
ros2 launch robot_bringup base.launch.py
```

### 4. Launch – SLAM (base + lidar + mapping)

```bash
ros2 launch robot_bringup slam.launch.py
```

To save a map after driving around:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 5. Launch – Navigation (base + lidar + Nav2 with pre-built map)

```bash
ros2 launch robot_bringup nav.launch.py map:=/home/pi/maps/my_map.yaml
```

## Modifying Parameters

### GPIO Pins (motor driver)

Edit `src/robot_driver/config/driver_params.yaml`:
- `gpio_chip`: `/dev/gpiochip4` (RPi 5) or `/dev/gpiochip0` (RPi 4)
- `motor_fl_in1`, `motor_fl_in2`, `motor_fl_ena`: Front-left motor pins
- Same pattern for FR, RL, RR

### GPIO Pins (encoders)

Edit `src/robot_odom/config/odom_params.yaml`:
- `encoder_fl_a`, `encoder_fl_b`: Front-left encoder A/B channels
- `single_phase`: set `true` if encoders have only one output

### I2C (BNO055)

Edit `src/robot_bno055/config/bno055_params.yaml`:
- `i2c_bus`: Usually `/dev/i2c-1`
- `i2c_address`: `0x28` (default) or `0x29`
- `operation_mode`: `0x0C` (NDOF with mag) or `0x08` (IMU, no mag)

### Serial Port (RPLIDAR)

Pass as launch argument:
```bash
ros2 launch robot_bringup slam.launch.py serial_port:=/dev/ttyUSB1
```

## Verification Commands

### Check Topics

```bash
ros2 topic list
ros2 topic echo /wheel/odometry         # Wheel odometry
ros2 topic echo /imu/data               # IMU data
ros2 topic echo /scan                   # Lidar scan
ros2 topic echo /odometry/filtered      # EKF output
```

### Check TF Tree

```bash
ros2 run tf2_tools view_frames          # Generates frames.pdf
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

### Check IMU Calibration

```bash
# BNO055 calibration status is logged by the node.
# Look for "BNO055 calibration" in the terminal output.
# Values 0–3: 0=uncalibrated, 3=fully calibrated
```

### Check Encoders

```bash
ros2 topic echo /wheel/left_vel
ros2 topic echo /wheel/right_vel
ros2 topic echo /joint_states
```

### Teleop Test

```bash
# Install teleop if needed:
apt-get install ros-jazzy-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Troubleshooting

### "Permission denied" on GPIO

The Docker container needs access to GPIO devices:
```bash
docker run --privileged \
  --device /dev/gpiochip4 \
  --device /dev/i2c-1 \
  --device /dev/ttyUSB0 \
  ...
```

Or add specific device access:
```bash
docker run --device /dev/gpiochip4 --device /dev/i2c-1 --device /dev/ttyUSB0 ...
```

### BNO055 not detected

```bash
# Check I2C bus:
i2cdetect -y 1
# Should show 0x28 or 0x29

# If not visible:
# 1. Check wiring (SDA=GPIO2, SCL=GPIO3 on RPi)
# 2. Enable I2C: raspi-config → Interface → I2C
# 3. Check device tree overlay in /boot/config.txt: dtparam=i2c_arm=on
```

### RPLIDAR not spinning

```bash
# Check serial device:
ls -la /dev/ttyUSB*

# Check permissions:
chmod 666 /dev/ttyUSB0
# Or add user to dialout group
```

### Motors not moving

1. Check GPIO chip: `gpiodetect` (should show gpiochip4 on RPi 5)
2. Check pin access: `gpioinfo gpiochip4`
3. Run with `open_loop: true` first to test without encoders
4. Check driver debug topics:
   ```bash
   ros2 topic echo /driver/left_pwm
   ros2 topic echo /driver/right_pwm
   ```

### EKF not publishing

```bash
# Check if wheel odom and IMU are being received:
ros2 topic hz /wheel/odometry
ros2 topic hz /imu/data

# Both must be publishing for EKF to produce output.
```
