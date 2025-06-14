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

💡 古いGazeboはここっから

## インストール

```
sudo apt-get install -y \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-slam-toolbox \
ros-humble-teleop-tools \
ros-humble-cartographer \
ros-humble-cartographer-ros \
ros-humble-dynamixel-sdk \
ros-humble-xacro \
ros-humble-ament-cmake-clang-format \
ros-humble-rmw-cyclonedds-cpp

sudo apt-get install -y python3-numpy python3-matplotlib python3-seaborn

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=happy_mini" >> ~/.bashrc

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs
git clone https://github.com/AI-Robot-Book-Humble/turtlebot3_happy_mini.git src/turtlebot3_happy_mini
git clone https://github.com/owhinata/happy_nav src/happy_nav

. /opt/ros/humble/setup.bash
rosdep install --default-yes --from-paths src --ignore-src
colcon build --symlink-install
. install/setup.bash
```

## シミュレータの起動

```bash
. /usr/share/gazebo/setup.bash

# ↓ のどれか
ros2 launch turtlebot3_gazebo empty_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_house2.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage2.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage3.launch.py
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
```

## Nodes
- teleop_twist_keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- [happy_teleop_node](happy_nav/happy_teleop_node.py)
```bash
ros2 run happy_nav happy_teleop_node
```

- [happy_move_node](happy_nav/happy_move_node.py)
```bash
ros2 run happy_nav happy_move_node
```

<details><summary>⚠ numpyが1系でないと tf_transformations.euler_from_quaternionがエラーになる</summary>

```bash
$ ros2 run happy_nav happy_move_node 
Traceback (most recent call last):
  File "/home/ouwa/work/ros2_ws/install/happy_nav/lib/happy_nav/happy_move_node", line 33, in <module>
    sys.exit(load_entry_point('happy-nav', 'console_scripts', 'happy_move_node')())
  File "/home/ouwa/work/ros2_ws/install/happy_nav/lib/happy_nav/happy_move_node", line 25, in importlib_load_entry_point
    return next(matches).load()
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 171, in load
    module = import_module(match.group('module'))
  File "/usr/lib/python3.10/importlib/__init__.py", line 126, in import_module
    return _bootstrap._gcd_import(name[level:], package, level)
  File "<frozen importlib._bootstrap>", line 1050, in _gcd_import
  File "<frozen importlib._bootstrap>", line 1027, in _find_and_load
  File "<frozen importlib._bootstrap>", line 1006, in _find_and_load_unlocked
  File "<frozen importlib._bootstrap>", line 688, in _load_unlocked
  File "<frozen importlib._bootstrap_external>", line 883, in exec_module
  File "<frozen importlib._bootstrap>", line 241, in _call_with_frames_removed
  File "/home/ouwa/work/ros2_ws/build/happy_nav/happy_nav/happy_move_node.py", line 3, in <module>
    import tf_transformations
  File "/opt/ros/humble/lib/python3.10/site-packages/tf_transformations/__init__.py", line 46, in <module>
    import transforms3d
  File "/usr/lib/python3/dist-packages/transforms3d/__init__.py", line 10, in <module>
    from . import quaternions
  File "/usr/lib/python3/dist-packages/transforms3d/quaternions.py", line 26, in <module>
    _MAX_FLOAT = np.maximum_sctype(np.float)
  File "/home/ouwa/.local/lib/python3.10/site-packages/numpy/__init__.py", line 400, in __getattr__
    raise AttributeError(
AttributeError: `np.maximum_sctype` was removed in the NumPy 2.0 release. Use a specific dtype instead. You should avoid relying on any implicit mechanism and select the largest dtype of a kind explicitly in the code.
[ros2run]: Process exited with failure 1
```
</details>

pip3でインストールするとたぶん2系が入るので aptで入れ直す。



