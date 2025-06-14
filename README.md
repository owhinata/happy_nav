# happy_nav
ros2_ai_robot_ch4

💡 これは Ignition

### バージョン系譜

| Ubuntu | ROS2 | Gazebo |
| -- | -- | -- |
| 22.04 | Humble | Fortress |
| 24.04 | Jazzy | Harmonic |

豆知識
Gazeboは昔はGazeboだった。
2020年ころにリアーキでIgnitionになったが、混乱もあり再度Gazeboの名称に戻った
そのため Ignition Fortress, Gazebo Harmonic となっている

## インストール手順 (Ubuntu 22.04 - Humble - Fortress)

👉 https://gazebosim.org/docs/fortress/install_ubuntu/

```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
sudo apt-get install ros-humble-ros-ign-bridge
```

## 使い方

👉 https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html#

```bash
# 起動
ign gazebo -v 4 -r visualize_lidar.sdf

# Topic確認
ign topic -l

# ROSのTopic確認
ros2 topic list

# 以上からGazeboとROSのTopicは無関係であることがわかる
# → ブリッジでGazeboのTopicをROSのTopicに変換する

# ブリッジのインストール
sudo apt-get install ros-humble-ros-ign-bridge

# ブリッジノードの起動
. /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist

# ROSからTwistメッセージをpublishしてみる → 2輪差動ロボが前進する
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"

# ROSのteleopパッケージをインストール
sudo apt-get install ros-humble-teleop-twist-keyboard

# ROSのTeleopノードで2輪差動ロボを動かす
. /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel

# GazeboのLidarトピックをRvizで確認する
. /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan

. /opt/ros/humble/setup.bash
rviz2 # Global Options > Fixed frame: vehicle_blue/lidar_link/gpu_lidar, Add Topic: /laser_scan
```
