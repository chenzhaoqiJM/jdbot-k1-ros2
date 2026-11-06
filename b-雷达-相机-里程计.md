# 里程计

## 雷达说明

默认箭头标注方向为x正向，tf 的旋转全 0

```bash
ros2 launch jdbot_base_bringup start_lslidar_n10p.launch.py
```

ydlidar x3 pro，该配置为电机在后

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

使箭头标注方向为x正向，tf 的旋转全 0

```bash
ros2 launch jdbot_base_bringup start_rplidar.launch.py serial_port:=/dev/ttyUSB0 flip_x_axis:=true frame_id:=rplidar_link
```


## 雷达里程计

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

```bash
ros2 launch jdbot_base_bringup start_rplidar.launch.py serial_port:=/dev/ttyUSB1 flip_x_axis:=true frame_id:=rplidar_link
```

```bash
ros2 launch cartographer_lidar_odometry lidar_odometry.launch.py
```

## 雷达里程计2

```bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py
```

配合测试对比

```bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py odom_topic:=/odom_self publish_tf:=false
```

```bash
ros2 run jdbot_base_bringup orbslam3_odom_to_tf --ros-args -p odom_topic:=/odom_self -p parent_frame:=odom -p child_frame:=base_footprint_self
```