// Copyright 2026 SpacemiT
// Licensed under the Apache License, Version 2.0.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

// POSIX serial I/O keeps this package independent of a third-party serial library.
class SerialPort
{
public:
  SerialPort(const std::string & port, int64_t baudrate)
  {
    const auto speed = baud_constant(baudrate);
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Cannot open " + port + ": " + std::strerror(errno));
    }
    termios settings{};
    if (tcgetattr(fd_, &settings) != 0) {
      fail_setup();
    }
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(CSTOPB | PARENB | CSIZE | CRTSCTS);
    settings.c_cflag |= CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, speed) != 0 || cfsetospeed(&settings, speed) != 0 ||
      tcsetattr(fd_, TCSANOW, &settings) != 0 || tcflush(fd_, TCIFLUSH) != 0)
    {
      fail_setup();
    }
  }

  ~SerialPort() {::close(fd_);}
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  ssize_t read(char * data, size_t size)
  {
    pollfd descriptor{fd_, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 50);  // Python serial timeout: 0.05 s.
    if (ready < 0 && errno == EINTR) {
      return 0;
    }
    if (ready < 0) {
      throw std::runtime_error(std::strerror(errno));
    }
    if (ready == 0) {
      return 0;
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error("Serial device disconnected");
    }
    const auto count = ::read(fd_, data, size);
    if (count < 0 && (errno == EAGAIN || errno == EINTR)) {
      return 0;
    }
    if (count < 0) {
      throw std::runtime_error(std::strerror(errno));
    }
    return count;
  }

  void write(const std::string & data)
  {
    size_t offset = 0;
    while (offset < data.size()) {
      const auto count = ::write(fd_, data.data() + offset, data.size() - offset);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count < 0 && errno == EAGAIN) {
        pollfd descriptor{fd_, POLLOUT, 0};
        if (::poll(&descriptor, 1, 50) > 0 && descriptor.revents & POLLOUT) {
          continue;
        }
        throw std::runtime_error("Serial write timed out");
      }
      if (count <= 0) {
        throw std::runtime_error("Serial write failed: " + std::string(std::strerror(errno)));
      }
      offset += static_cast<size_t>(count);
    }
  }

private:
  void fail_setup()
  {
    const std::string message = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("Cannot configure serial port: " + message);
  }

  static speed_t baud_constant(int64_t baudrate)
  {
    switch (baudrate) {
      case 50: return B50;
      case 75: return B75;
      case 110: return B110;
      case 134: return B134;
      case 150: return B150;
      case 200: return B200;
      case 300: return B300;
      case 600: return B600;
      case 1200: return B1200;
      case 1800: return B1800;
      case 2400: return B2400;
      case 4800: return B4800;
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 500000: return B500000;
      case 576000: return B576000;
      case 921600: return B921600;
      case 1000000: return B1000000;
      case 1152000: return B1152000;
      case 1500000: return B1500000;
      case 2000000: return B2000000;
      case 2500000: return B2500000;
      case 3000000: return B3000000;
      case 3500000: return B3500000;
      case 4000000: return B4000000;
      default:
        throw std::invalid_argument("Unsupported serial baudrate: " + std::to_string(baudrate));
    }
  }

  int fd_{-1};
};

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n\v\f");
  return begin == std::string::npos ? "" :
         text.substr(begin, text.find_last_not_of(" \t\r\n\v\f") - begin + 1);
}

std::vector<std::string> split(const std::string & text, char delimiter)
{
  std::vector<std::string> parts;
  size_t begin = 0;
  size_t end;
  while ((end = text.find(delimiter, begin)) != std::string::npos) {
    parts.push_back(text.substr(begin, end - begin));
    begin = end + 1;
  }
  parts.push_back(text.substr(begin));
  return parts;
}

double parse_double(const std::string & text)
{
  const auto value = trim(text);
  size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size()) {
    throw std::invalid_argument("Invalid float: " + text);
  }
  return result;
}

int64_t parse_integer(const std::string & text)
{
  const auto value = trim(text);
  size_t used = 0;
  const auto result = std::stoll(value, &used);
  if (used != value.size()) {
    throw std::invalid_argument("Invalid integer: " + text);
  }
  return result;
}
}  // namespace

// Keep control, feedback and integration semantics aligned with ros2_ctrl_odom_node.py.
class CmdVelToSerial : public rclcpp::Node
{
public:
  CmdVelToSerial()
  : Node("cmdvel_to_serial")
  {
    const auto port = declare_parameter<std::string>("serial_port", "/dev/jdbot");
    const auto baud = declare_parameter<int64_t>("baudrate", 115200);
    const auto send_hz = declare_parameter<double>("send_hz", 20.0);
    const auto odom_hz = declare_parameter<double>("odom_hz", 50.0);
    timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.4);
    cmd_vel_deadzone_threshold_ = declare_parameter<double>("cmd_vel_deadzone_threshold", 0.001);
    min_linear_speed_ = declare_parameter<double>("min_linear_speed", 0.05);
    min_angular_speed_ = declare_parameter<double>("min_angular_speed", 0.26);
    wheel_diameter_ = declare_parameter<double>("wheel_diameter", 0.067);
    wheel_base_ = declare_parameter<double>("wheel_base", 0.183);
    motor1_factor_ = declare_parameter<double>("motor1_factor", 1.0);
    motor2_factor_ = declare_parameter<double>("motor2_factor", 1.0);
    feedback_pwm_deadzone_ = declare_parameter<int64_t>("feedback_pwm_deadzone", 0);
    const auto encoder_ppr = declare_parameter<double>("encoder_ppr", 1000.0);
    const auto reduction_ratio = declare_parameter<double>("reduction_ratio", 56.0);
    const auto ff_factor = declare_parameter<double>("ff_factor", 310.0);
    const auto pid_kp = declare_parameter<double>("pid_kp", 10.0);
    const auto pid_ki = declare_parameter<double>("pid_ki", 70.0);
    const auto pid_kd = declare_parameter<double>("pid_kd", 0.0);
    const auto straight_balance_kp = declare_parameter<double>("straight_balance_kp", 25.0);
    const auto straight_balance_ki = declare_parameter<double>("straight_balance_ki", 8.0);
    const auto straight_balance_kd = declare_parameter<double>("straight_balance_kd", 0.0);
    const auto cfg_send_on_startup = declare_parameter<bool>("cfg_send_on_startup", true);
    debug_ = declare_parameter<bool>("debug", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    if (!std::isfinite(send_hz) || send_hz <= 0.0 ||
      !std::isfinite(odom_hz) || odom_hz <= 0.0 ||
      !std::isfinite(wheel_diameter_) || wheel_diameter_ <= 0.0 ||
      !std::isfinite(wheel_base_) || wheel_base_ <= 0.0)
    {
      throw std::invalid_argument("send_hz, odom_hz, wheel_diameter and wheel_base must be positive");
    }

    serial_ = std::make_unique<SerialPort>(port, baud);
    RCLCPP_INFO(get_logger(), "Serial opened: %s @ %ld", port.c_str(), static_cast<long>(baud));
    if (cfg_send_on_startup) {
      std::ostringstream cfg;
      cfg.imbue(std::locale::classic());
      cfg << std::fixed << std::setprecision(3) << "CFG," << encoder_ppr << ',' <<
        reduction_ratio << ',' << ff_factor << ',' << pid_kp << ',' << pid_ki << ',' <<
        pid_kd << ',' << straight_balance_kp << ',' << straight_balance_ki << ',' <<
        straight_balance_kd << '\n';
      serial_->write(cfg.str());
      RCLCPP_INFO(get_logger(), "Sent CFG: %s", trim(cfg.str()).c_str());
    }

    last_cmd_time_ = SystemClock::now();
    last_odom_time_ = SteadyClock::now();
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> guard(lock_);
        cur_v_ = msg->linear.x;
        cur_w_ = msg->angular.z;
        last_cmd_time_ = SystemClock::now();
      });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // Like rclpy.create_timer, use the ROS clock for scheduling, while odom dt is monotonic.
    send_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / send_hz),
      [this]() {send_timer_cb();});
    odom_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / odom_hz),
      [this]() {odom_timer_cb();});
    read_thread_ = std::thread([this]() {serial_read_thread();});
    RCLCPP_INFO(get_logger(), "cmd_vel → serial + odom node started");
  }

  ~CmdVelToSerial() override
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    // SerialPort closes only after the reader has stopped.
  }

private:
  double apply_cmd_vel_deadzone(double value, double min_abs_value) const
  {
    const auto magnitude = std::abs(value);
    if (cmd_vel_deadzone_threshold_ < magnitude && magnitude < min_abs_value) {
      return std::copysign(min_abs_value, value);
    }
    return value;
  }

  std::pair<int, double> wheel_speed_to_cmd(double velocity) const
  {
    if (std::abs(velocity) < 1e-3) {
      return {0, 0.0};
    }
    return {velocity > 0.0 ? 1 : 2, std::abs(velocity) / (kPi * wheel_diameter_)};
  }

  void send_timer_cb()
  {
    double v = 0.0;
    double w = 0.0;
    {
      std::lock_guard<std::mutex> guard(lock_);
      if (std::chrono::duration<double>(SystemClock::now() - last_cmd_time_).count() <= timeout_) {
        v = cur_v_;
        w = cur_w_;
      }
    }
    v = apply_cmd_vel_deadzone(v, min_linear_speed_);
    w = apply_cmd_vel_deadzone(w, min_angular_speed_);
    const auto left = wheel_speed_to_cmd(v - w * wheel_base_ / 2.0);
    const auto right = wheel_speed_to_cmd(v + w * wheel_base_ / 2.0);
    std::ostringstream cmd;
    cmd.imbue(std::locale::classic());
    cmd << std::fixed << std::setprecision(2) << left.first << ',' <<
      left.second * motor1_factor_ << ';' << right.first << ',' <<
      right.second * motor2_factor_ << '\n';
    serial_->write(cmd.str());
  }

  double parse_motor(const std::string & text) const
  {
    const auto items = split(text, ',');
    if (items.size() < 3) {
      throw std::invalid_argument("Motor feedback needs speed,direction,pwm");
    }
    const auto pwm = parse_integer(items.back());
    auto speed = parse_double(items[items.size() - 3]);
    const auto direction = parse_integer(items[items.size() - 2]);
    if (std::abs(pwm) < feedback_pwm_deadzone_) {
      return 0.0;
    }
    // Feedback direction is intentionally opposite to the outgoing command convention.
    if (direction == 1) {
      speed = -speed;
    } else if (direction == 0) {
      speed = 0.0;
    }
    return speed * kPi * wheel_diameter_;
  }

  void parse_feedback(const std::string & text)
  {
    try {
      const auto parts = split(text, ';');
      if (parts.size() != 3) {
        throw std::invalid_argument("Expected voltage;left;right");
      }
      const auto v_l = parse_motor(parts[1]);
      const auto v_r = parse_motor(parts[2]);
      const auto voltage = parse_double(parts[0]);
      if (debug_) {
        const auto left = split(parts[1], ',');
        const auto right = split(parts[2], ',');
        const auto speed_l = parse_double(left[left.size() - 3]);
        const auto speed_r = parse_double(right[right.size() - 3]);
        const auto ff_l = std::abs(speed_l) > 1e-3 ? parse_double(left.back()) / speed_l : 0.0;
        const auto ff_r = std::abs(speed_r) > 1e-3 ? parse_double(right.back()) / speed_r : 0.0;
        RCLCPP_INFO(
          get_logger(), "left:%s, right:%s,速度差:%.2f, 左轮ff_factor:%.2f, 右轮ff_factor:%.2f",
          parts[1].c_str(), parts[2].c_str(), speed_r - speed_l, ff_l, ff_r);
      }
      std::lock_guard<std::mutex> guard(lock_);
      v_l_ = v_l;
      v_r_ = v_r;
      power_voltage_ = voltage;
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Parse error: %s: %s", text.c_str(), error.what());
    }
  }

  void serial_read_thread()
  {
    std::string buffer;
    RCLCPP_INFO(get_logger(), "Serial RX thread started");
    while (rclcpp::ok() && running_) {
      try {
        char data[64];
        const auto count = serial_->read(data, sizeof(data));
        if (count == 0) {
          continue;
        }
        buffer.append(data, static_cast<size_t>(count));
        size_t end;
        while ((end = buffer.find('\n')) != std::string::npos) {
          const auto text = trim(buffer.substr(0, end));
          buffer.erase(0, end + 1);
          if (text.empty()) {
            continue;
          }
          if (text.rfind("CFG_OK", 0) == 0) {
            RCLCPP_INFO(get_logger(), "CFG ACK: %s", text.c_str());
          } else if (text.rfind("CFG_ERR", 0) == 0) {
            RCLCPP_ERROR(get_logger(), "CFG error: %s", text.c_str());
          } else {
            parse_feedback(text);
          }
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Serial read error: %s", error.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void odom_timer_cb()
  {
    const auto now = SteadyClock::now();
    const double dt = std::chrono::duration<double>(now - last_odom_time_).count();
    last_odom_time_ = now;
    if (dt <= 0.0) {
      return;
    }
    double v_l;
    double v_r;
    {
      std::lock_guard<std::mutex> guard(lock_);
      v_l = v_l_;
      v_r = v_r_;
    }
    const auto v = (v_r + v_l) / 2.0;
    const auto w = (v_r - v_l) / wheel_base_;
    // Python updates yaw first, then integrates position with the new yaw.
    yaw_ += w * dt;
    x_ += v * std::cos(yaw_) * dt;
    y_ += v * std::sin(yaw_) * dt;
    publish_odom(v, w);
  }

  void publish_odom(double v, double w)
  {
    const auto stamp = get_clock()->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub_->publish(odom);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::unique_ptr<SerialPort> serial_;
  std::mutex lock_;
  std::atomic<bool> running_{true};
  std::thread read_thread_;
  double timeout_, cmd_vel_deadzone_threshold_, min_linear_speed_, min_angular_speed_;
  double wheel_diameter_, wheel_base_, motor1_factor_, motor2_factor_;
  int64_t feedback_pwm_deadzone_;
  bool debug_, publish_tf_;
  std::string odom_frame_, base_frame_;
  double cur_v_{0.0}, cur_w_{0.0}, v_l_{0.0}, v_r_{0.0}, power_voltage_{0.0};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  SystemClock::time_point last_cmd_time_;
  SteadyClock::time_point last_odom_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr send_timer_, odom_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<CmdVelToSerial>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cmdvel_to_serial"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
