## 安装依赖

```bash
sudo apt install python3-zmq python3-serial python3-transforms3d \
libspdlog-dev libconsole-bridge-dev liborocos-kdl-dev nlohmann-json3-dev liblgpio-dev \
liblttng-ust-dev libgpiod-dev ros-humble-camera-info-manager ros-humble-slam-toolbox \
ros-humble-cartographer ros-humble-cartographer-ros ros-humble-nav2* 'ros-humble-rtabmap*' \
ros-humble-aruco-markers-msgs ros-humble-realsense2-camera ros-dev-tools ros-humble-desktop \
udev git cmake ninja-build build-essential patchelf libboost-serialization-dev libssl-dev libsuitesparse-dev \
libglew-dev libepoxy-dev libjpeg-dev libpng-dev libtiff-dev wget \
libeigen3-dev
```

## 安装 YDLIDAR SDK

```bash
git clone https://github.com/chenzhaoqiJM/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
cmake --build . -- -j8
sudo cmake --install .
```

## 编译说明

下载

```bash
mkdir ~/jdbot_ws && cd ~/jdbot_ws
git clone https://github.com/chenzhaoqiJM/jdbot-k1-ros2.git
```

编译

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 组

```bash
sudo usermod -aG video $USER
sudo usermod -aG audio $USER
sudo usermod -aG dialout $USER
```