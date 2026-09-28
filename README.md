# Robot Odometry: EKF Sensor Fusion (ROS 2)

A ROS 2 (C++) node that fuses **LiDAR**, **visual**, **wheel** and **IMU** odometry into a single, smoother pose estimate using an **Extended Kalman Filter (EKF)**.

<!-- Add a GIF / screenshot / RViz plot here: fused path vs. individual sensors -->

## Why this exists

Every odometry source drifts or is noisy in its own way:

| Source | Strength | Weakness |
|---|---|---|
| Wheel | Smooth, high-rate | Slips, drifts over time |
| LiDAR (rf2o) | Good short-term x/y | Struggles in featureless spaces |
| Visual | Independent of the ground | Noisy, lighting dependent |
| IMU | Reliable yaw | No position, gyro drift |

This node weights each sensor by how much it is trusted (its measurement covariance) and produces one fused pose on `/position`.

## How it works

**State vector:** `[x, y, theta, bias]`

1. **Predict** (7 Hz timer): non-linear motion model using the commanded velocity (`/cmd_vel`) and IMU yaw, with the Jacobian `F_k` used to propagate the covariance.
2. **Update** (sequential, one sensor at a time):
   - LiDAR: measures `x + bias`, `y`, `theta`
   - Visual: measures `x + bias`, `y`, `theta`
   - IMU: measures `theta` only (angle wrapped with `atan2(sin, cos)`)
3. Covariance updates use the **Joseph form** for numerical stability.
4. Matrix math is done with [Blaze](https://bitbucket.org/blaze-lib/blaze) for speed.

The `bias` term models a constant offset in the measured x position.

## ROS 2 interface

**Node:** `Robot_Odometry_Node`

| Direction | Topic | Type | Description |
|---|---|---|---|
| Sub | `/odom_rf2o` | `nav_msgs/Odometry` | LiDAR odometry (e.g. from `rf2o_laser_odometry`) |
| Sub | `/visual_pose` | `geometry_msgs/Pose2D` | Visual odometry |
| Sub | `/wheel_pose` | `geometry_msgs/Pose2D` | Wheel odometry |
| Sub | `/imu_pose` | `geometry_msgs/Pose2D` | IMU yaw (`theta`) |
| Sub | `/cmd_vel` | `geometry_msgs/Twist` | Commanded velocity, used in the motion model |
| Pub | `/position` | `geometry_msgs/Pose2D` | Fused pose |

> **Note:** the published `y` and `theta` are sign-flipped to match the robot's TF / angle convention.

## Dependencies

- ROS 2 (**<your distro, e.g. Humble>**)
- `rclcpp`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2`, `tf2_ros`
- [Blaze](https://bitbucket.org/blaze-lib/blaze) (C++ math library)
- Optional: [`rf2o_laser_odometry`](https://github.com/MAPIRlab/rf2o_laser_odometry) to produce `/odom_rf2o`

## Build & run

```bash
# inside your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/<repo-name>.git
cd ~/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash

ros2 run <package_name> robot_odo
```

## Tuning

Sensor trust is set by the measurement noise values in the source (`usm_*`). **Higher variance means lower trust.**

| Parameter | Meaning |
|---|---|
| `usm_wheel_*`, `usm_lidar_*`, `usm_visual_*`, `usm_imu_theta` | Measurement uncertainty per sensor |
| `eta_*` | Initial state uncertainty |
| `eta_bias_process` | Process noise for the bias state |
| `time_period` | Filter rate (default 1/7 s) |

Two presets are included in comments: **Droid** (active) and **AGV**. Swap them depending on your platform.

## Robot geometry (TF)

Sensor offsets from `base_link` are defined in the code (LiDAR, camera, four wheels). The TF broadcaster is currently **commented out**; uncomment `tf_update_func` and the broadcaster setup if you want this node to publish TF.

## Known limitations / TODO

- [ ] Parameters are hardcoded; move them to ROS 2 parameters / a YAML file
- [ ] `/cmd_vel` is used as the velocity input instead of measured velocity
- [ ] LiDAR and wheel `theta` are replaced by IMU yaw, so heading is effectively IMU driven
- [ ] Measurement residuals are computed from the predicted state, not the state after the previous sensor's update
- [ ] No message timestamp synchronization between sensors
- [ ] Add launch file, unit tests, and example rosbag

## Author

**Pratham Mittal**
prathammittal2411@gmail.com

Created: 9 Aug 2025 · Last modified: 3 Jan 2026

## License

Add a license (e.g. MIT or Apache-2.0) and put it in a `LICENSE` file.
