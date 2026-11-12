# 简易小车快速启动命令

系统版本：Bianbu 2.3.5
ROS2 版本：Humble

PC Linux 版本：Ubuntu22.04，ROS2 Humble

环境配置：K1参考 installation.md、PC 安装 ROS2 Humble 并编译 visualization 包

## 建图

### 启动底盘

```bash
ros2 launch jdbot_base_bringup start_control_diffcar_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.169 wheel_diameter:=0.065 min_angular_speed:=0.15 min_linear_speed:=0.01 use_cpp:=true
```

### 启动雷达

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

### 启动建图

```bash
ros2 launch jdbot_slam cartographer.launch.py
```

### 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### PC 可视化

同一局域网

```bash
ros2 launch visualization display_slam.launch.py
```

### 保存

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 导航

### 复制地图文件

```bash
cp my_map.* ~/jdbot_ws/jdbot-k1-ros2/jdbot_navigation/map/
cd ~/jdbot_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select jdbot_navigation
```

### 启动底盘

```bash
ros2 launch jdbot_base_bringup start_control_diffcar_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.169 wheel_diameter:=0.065 min_angular_speed:=0.15 min_linear_speed:=0.01 use_cpp:=true
```

### 启动雷达

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

### 启动里程计

```bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py
```

### 启动导航

```bash
ros2 launch jdbot_navigation lidar_nav2.launch.py
```

### PC 可视化

同一局域网

```bash
ros2 launch visualization display_navigation.launch.py
```

## 跟随

### 启动底盘

```bash
ros2 launch jdbot_base_bringup start_control_diffcar_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.169 wheel_diameter:=0.065 min_angular_speed:=0.15 min_linear_speed:=0.01 use_cpp:=true
```

### 启动相机

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video20"
```

### 启动跟随

```bash
ros2 launch jobot_mono_follow agv_person_follow.launch.py
```

```bash
ros2 service call /toggle_follow std_srvs/srv/SetBool "{data: true}"
```
