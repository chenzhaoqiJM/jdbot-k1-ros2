cmake_minimum_required(VERSION 3.8)
project(jdbot_esp32_c6_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

add_executable(ros2_ctrl_odom_node src/ros2_ctrl_odom_node.cpp)
target_compile_features(ros2_ctrl_odom_node PUBLIC cxx_std_17)
ament_target_dependencies(ros2_ctrl_odom_node rclcpp geometry_msgs nav_msgs tf2_ros)
target_link_libraries(ros2_ctrl_odom_node Threads::Threads)

install(TARGETS ros2_ctrl_odom_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(serial_odom test/test_serial_odom.py
    ENV "ESP32_CONTROL_EXECUTABLE=$<TARGET_FILE:ros2_ctrl_odom_node>"
    TIMEOUT 60)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
