# Copyright 2026 SpacemiT
# Licensed under the Apache License, Version 2.0.

"""Exercise the real executable through a pseudo terminal and ROS topics."""

import math
import os
import select
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from tf2_msgs.msg import TFMessage


class Controller:

    def __init__(self, directory, **parameters):
        # Isolate test traffic from any robot running on the host.
        os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 50)
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        self.context = Context()
        rclpy.init(context=self.context)
        self.namespace = 'esp32_test_' + uuid.uuid4().hex[:8]
        self.node = rclpy.create_node('probe', namespace=self.namespace, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.odometry = []
        self.transforms = []
        self.node.create_subscription(Odometry, 'wheel_odom', self.odometry.append, 100)
        self.node.create_subscription(TFMessage, '/tf', self.transforms.append, 100)
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.master, self.slave = os.openpty()
        self.buffer = b''
        self.lines = []
        self.log_path = directory / 'controller.log'
        self.log_file = self.log_path.open('w')
        params = {
            'serial_port': os.ttyname(self.slave),
            'odom_topic': 'wheel_odom',
            'odom_frame': self.namespace + '_odom',
            'base_frame': self.namespace + '_base',
            **parameters,
        }
        command = [os.environ['ESP32_CONTROL_EXECUTABLE'], '--ros-args',
                   '-r', '__ns:=/' + self.namespace]
        for key, value in params.items():
            command += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
        self.process = subprocess.Popen(command, stdout=self.log_file, stderr=subprocess.STDOUT)

    def pump(self):
        self.executor.spin_once(timeout_sec=0.01)
        if select.select([self.master], [], [], 0)[0]:
            self.buffer += os.read(self.master, 4096)
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                self.lines.append(line.decode().strip())
        assert self.process.poll() is None, self.log_path.read_text()

    def until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                return
        pytest.fail(f'Controller timed out: {self.log_path.read_text()}')

    def start(self):
        self.until(lambda: self.odometry and self.publisher.get_subscription_count() == 1)

    def command(self, v, w, expected):
        self.lines.clear()
        message = Twist()
        message.linear.x = v
        message.angular.z = w
        self.publisher.publish(message)
        self.until(lambda: expected in self.lines)

    def feedback(self, text, v, w):
        self.odometry.clear()
        os.write(self.master, text.encode())
        self.until(lambda: any(
            math.isclose(m.twist.twist.linear.x, v, abs_tol=1e-10)
            and math.isclose(m.twist.twist.angular.z, w, abs_tol=1e-10)
            for m in self.odometry))

    def close(self):
        try:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=3)
            assert self.process.returncode == 0, self.log_path.read_text()
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait()
            self.log_file.close()
            os.close(self.master)
            os.close(self.slave)
            self.node.destroy_node()
            self.executor.shutdown()
            self.context.shutdown()


@pytest.fixture
def controller(tmp_path):
    instances = []

    def create(**parameters):
        instance = Controller(tmp_path, **parameters)
        instances.append(instance)
        instance.start()
        return instance

    yield create
    for instance in instances:
        instance.close()


def test_configuration_commands_and_timeout(controller):
    driver = controller()
    assert 'CFG,1000.000,56.000,310.000,10.000,70.000,0.000,25.000,8.000,0.000' in driver.lines
    for v, w, expected in [
        (0.02, 0.0, '1,0.24;1,0.24'),
        (-0.02, 0.0, '2,0.24;2,0.24'),
        (0.0, 0.01, '2,0.11;1,0.11'),
        (0.0, -0.01, '1,0.11;2,0.11'),
        (0.001, 0.0, '1,0.00;1,0.00'),
        (0.0009, 0.0, '0,0.00;0,0.00'),
        (0.05, 0.26, '1,0.12;1,0.35'),
        (0.08, -0.3, '1,0.51;1,0.25'),
    ]:
        driver.command(v, w, expected)
    driver.lines.clear()
    driver.until(lambda: '0,0.00;0,0.00' in driver.lines, timeout=1.0)


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def test_feedback_odometry_tf_and_invalid_lines(controller):
    driver = controller(debug=True)
    circumference = math.pi * 0.067
    os.write(driver.master, b'CFG_OK\r\nCFG_ERR,test\n\n12.5;100,1.0,2,0;')
    driver.feedback('200,1.0,2,0\n', circumference, 0.0)
    # A PWM of zero is still valid encoder feedback with the default deadzone.
    driver.odometry.clear()
    driver.until(lambda: len(driver.odometry) >= 5)
    first, last = driver.odometry[0], driver.odometry[-1]
    dt = stamp_seconds(last) - stamp_seconds(first)
    assert last.pose.pose.position.x - first.pose.pose.position.x == pytest.approx(
        circumference * dt, abs=0.001)
    assert last.pose.pose.position.y == 0.0
    assert last.pose.pose.orientation.w == 1.0
    assert last.header.frame_id == driver.namespace + '_odom'
    assert last.child_frame_id == driver.namespace + '_base'
    assert all(value == 0.0 for value in last.pose.covariance)
    assert all(value == 0.0 for value in last.twist.covariance)

    # Unequal feedback exercises the Python yaw-first Euler integration order.
    v = 1.5 * circumference
    w = circumference / 0.183
    driver.feedback('12.5;100,1.0,2,90;200,2.0,2,100\n', v, w)
    driver.odometry.clear()
    driver.until(lambda: len(driver.odometry) >= 6)
    for before, after in zip(driver.odometry, driver.odometry[1:]):
        dt = stamp_seconds(after) - stamp_seconds(before)
        yaw = 2.0 * math.atan2(after.pose.pose.orientation.z, after.pose.pose.orientation.w)
        assert after.pose.pose.position.x - before.pose.pose.position.x == pytest.approx(
            v * math.cos(yaw) * dt, abs=0.0002)
        assert after.pose.pose.position.y - before.pose.pose.position.y == pytest.approx(
            v * math.sin(yaw) * dt, abs=0.0002)
    by_stamp = {stamp_seconds(m): m for m in driver.odometry}
    matching = [t for msg in driver.transforms for t in msg.transforms
                if t.header.frame_id == driver.namespace + '_odom'
                and stamp_seconds(t) in by_stamp]
    assert matching
    for transform in matching:
        odom = by_stamp[stamp_seconds(transform)]
        assert transform.transform.translation.x == odom.pose.pose.position.x
        assert transform.transform.translation.y == odom.pose.pose.position.y
        assert transform.transform.rotation == odom.pose.pose.orientation
        assert transform.child_frame_id == odom.child_frame_id

    # Bad records must leave the previous feedback intact and allow subsequent records.
    driver.feedback('broken\n12;1,1.0,2,10oops;2,2.0,2,20\n', v, w)
    driver.until(lambda: 'Parse error:' in driver.log_path.read_text())
    driver.feedback('12.5;100,1.0,1,-90;200,1.0,1,90\n', -circumference, 0.0)
    driver.feedback('12.5;100,1.0,0,90;200,1.0,0,90\n', 0.0, 0.0)
    log = driver.log_path.read_text()
    assert 'CFG ACK:' in log
    assert 'CFG error:' in log
    assert 'ff_factor:' in log


def test_overrides_and_disabled_cfg_tf(controller):
    driver = controller(
        cfg_send_on_startup=False, publish_tf=False,
        wheel_diameter=0.1, wheel_base=0.2, motor1_factor=2.0, motor2_factor=0.5,
        min_linear_speed=0.03, min_angular_speed=0.2, cmd_vel_deadzone_threshold=0.002,
        feedback_pwm_deadzone=10, send_hz=25.0, odom_hz=40.0)
    assert not any(line.startswith('CFG,') for line in driver.lines)
    driver.command(0.01, 0.0, '1,0.19;1,0.05')
    driver.command(0.0, 0.01, '2,0.13;1,0.03')
    driver.command(0.002, 0.0, '1,0.01;1,0.00')
    circumference = math.pi * 0.1
    driver.feedback('12;1,1.0,2,9;2,1.0,2,-9\n', 0.0, 0.0)
    driver.feedback('12;1,1.0,2,10;2,1.0,2,-10\n', circumference, 0.0)
    assert not any(t.header.frame_id == driver.namespace + '_odom'
                   for msg in driver.transforms for t in msg.transforms)
