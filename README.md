# CXD5602PWBIMU ROS2 Driver
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)

ROS2 package for [CXD5602PWBIMU](https://developer.sony.com/ja/spresense/specifications), a Multi-IMU for Sony Spresense.

## How to use 🔍
### Setup ⚙️
```bash
cd ~/ros2_ws/src
git clone https://github.com/shunsugar/cxd5602pwbimu.git
wstool merge cxd5602pwbimu/cxd5602pwbimu.rosinstall
wstool update
rosdep install -r -y -i --from-paths .
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Run 🚀
```
# Run node directly
ros2 run cxd5602pwbimu cxd5602pwbimu_node

# Or launch with default configuration
ros2 launch cxd5602pwbimu cxd5602pwbimu.launch.xml use_madgwick:=false

# Launch with Madgwick filter enabled
ros2 launch cxd5602pwbimu cxd5602pwbimu.launch.xml use_madgwick:=true
```

## License 🪪
Apache License 2.0 - see [LICENSE](https://github.com/shunsugar/cxd5602pwbimu?tab=Apache-2.0-1-ov-file) for details.

## IMU-only odometry `/odom/onlyimu`

`onlyimu_odom` estimates planar odometry using only the Spresense IMU. It does not subscribe to `/odom` or any wheel topic.

- input: `/imu/spresense/inityaw`
- output: `/odom/onlyimu`
- translation: accelerometer bias removal -> body-to-odom rotation -> gravity removal -> double integration
- orientation: input IMU quaternion
- drift reduction: low-pass filtering and acceleration deadband; ZUPT is available as an optional parameter but is disabled by default

The robot must remain stationary during the initial accelerometer calibration (`calibration_samples`, default: 960 samples). TF publication is disabled by default to avoid conflict with the existing wheel odometry TF.

Direct execution:

```bash
ros2 run cxd5602pwbimu onlyimu_odom --ros-args \
  -p imu_topic:=/imu/spresense/inityaw \
  -p odom_topic:=/odom/onlyimu
```
