# 底盘

ESP32-C6 的三个启动文件（`start_control_esp32_c6.launch.py`、`start_control_linglong_esp32_c6.launch.py`、`start_control_linksee_esp32_c6.launch.py`）均支持 `use_cpp` 参数：默认 `false` 使用原 Python 节点，设为 `true` 使用`jdbot_esp32_c6_control` 包中的 C++ 节点。两种实现使用相同的控制参数、话题和串口协议。

## linksee

```bash
ros2 launch jdbot_base_bringup start_control_linksee_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.185 wheel_diameter:=0.065 min_angular_speed:=0.4 min_angular_speed:=0.5
```

## 其他尺寸差速底盘

注意调整 tf 坐标系

```bash
ros2 launch jdbot_base_bringup start_control_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.270 wheel_diameter:=0.065 min_angular_speed:=0.4
```

## TF 坐标比对

```bash
ros2 run jdbot_base_bringup tf_pose_diff --ros-args -p second_frame:=base_footprint_rgbd
```
