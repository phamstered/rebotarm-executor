#!/usr/bin/env python3
"""Exercise the STM32 micro-ROS link at 100 Hz without enabling motors."""

import argparse
import time

import rclpy
from microros_stm32h723_bridge.msg import HardwareStatus, JointCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32MultiArray


class SessionLoadTest(Node):
    def __init__(self, rate_hz):
        super().__init__('stm32_microros_session_load_test')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.publisher = self.create_publisher(
            JointCommand, '/stm32/joint_commands', qos)
        self.create_subscription(
            JointState, '/stm32/joint_states', self._on_joint_state, qos)
        self.create_subscription(
            HardwareStatus, '/stm32/status', self._on_status, qos)
        self.create_subscription(
            UInt32MultiArray, '/stm32/microros_diagnostics',
            self._on_diagnostics, qos)
        self.period = 1.0 / rate_hz
        self.sequence = 1
        self.diagnostics = None
        self.joint_count = 0
        self.last_joint_time = None
        self.max_joint_gap = 0.0
        self.status_count = 0
        self.last_status_time = None
        self.max_status_gap = 0.0

    def _on_joint_state(self, _message):
        now = time.monotonic()
        if self.last_joint_time is not None:
            self.max_joint_gap = max(
                self.max_joint_gap, now - self.last_joint_time)
        self.last_joint_time = now
        self.joint_count += 1

    def _on_status(self, _message):
        now = time.monotonic()
        if self.last_status_time is not None:
            self.max_status_gap = max(
                self.max_status_gap, now - self.last_status_time)
        self.last_status_time = now
        self.status_count += 1

    def _on_diagnostics(self, message):
        if len(message.data) >= 14:
            self.diagnostics = list(message.data)

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.publisher.get_subscription_count() > 0 and
                    self.diagnostics is not None and self.joint_count > 0):
                if self.status_count == 0:
                    continue
                return
        raise RuntimeError('STM32 command subscriber/diagnostics unavailable')

    def _disable_command(self):
        message = JointCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.sequence = self.sequence
        self.sequence += 1
        message.mode = JointCommand.MODE_DISABLE
        message.position = [0.0] * 7
        message.velocity = [0.0] * 7
        message.kp = [0.0] * 7
        message.kd = [0.0] * 7
        message.effort = [0.0] * 7
        return message

    def run_load(self, duration):
        before = list(self.diagnostics)
        start_joint_count = self.joint_count
        start_status_count = self.status_count
        deadline = time.monotonic() + duration
        next_publish = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publisher.publish(self._disable_command())
                next_publish += self.period
                if next_publish < now - self.period:
                    next_publish = now + self.period
            rclpy.spin_once(self, timeout_sec=min(0.001, self.period / 4.0))

        load_end_joint_count = self.joint_count
        load_end_status_count = self.status_count
        settle_deadline = time.monotonic() + 1.2
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        after = list(self.diagnostics)
        received = load_end_joint_count - start_joint_count
        status_received = load_end_status_count - start_status_count

        labels = (
            'schema', 'session_generation', 'last_exit_reason',
            'last_spin_rc', 'last_ping_rc', 'consecutive_ping_failures',
            'consecutive_spin_failures', 'spin_failure_count',
            'ping_failure_count', 'publish_failure_count',
            'usb_rx_overflow_count', 'usb_write_timeout_count',
            'joint_state_publish_count', 'max_joint_state_publish_gap_ms',
            'usb_tx_overflow_count', 'usb_tx_high_water_mark')
        print('BEFORE ' + ', '.join(
            f'{name}={value}' for name, value in zip(labels, before)))
        print('AFTER  ' + ', '.join(
            f'{name}={value}' for name, value in zip(labels, after)))
        print(f'RECEIVED joint_states={received}, '
              f'rate={received / duration:.1f} Hz, '
              f'max_host_gap={self.max_joint_gap * 1000.0:.1f} ms')
        print(f'RECEIVED status={status_received}, '
              f'rate={status_received / duration:.1f} Hz, '
              f'max_host_gap={self.max_status_gap * 1000.0:.1f} ms')

        failures = []
        if after[1] != before[1]:
            failures.append('session_generation changed')
        if after[2] != 0:
            failures.append(f'last_exit_reason={after[2]}')
        for index, name in ((7, 'spin failures'), (8, 'ping failures'),
                            (10, 'USB RX overflows'),
                            (11, 'USB write timeouts')):
            if after[index] != before[index]:
                failures.append(
                    f'{name} increased {before[index]}->{after[index]}')
        for index, name in ((14, 'USB TX overflows'),):
            if len(after) > index and after[index] != before[index]:
                failures.append(
                    f'{name} increased {before[index]}->{after[index]}')
        if received < duration * 200.0:
            failures.append(f'joint state rate too low: {received / duration:.1f} Hz')
        if status_received < duration * 200.0:
            failures.append(
                f'status rate too low: {status_received / duration:.1f} Hz')
        if self.max_joint_gap > 0.1:
            failures.append(
                f'host joint state gap {self.max_joint_gap * 1000.0:.1f} ms')
        if self.max_status_gap > 0.1:
            failures.append(
                f'host status gap {self.max_status_gap * 1000.0:.1f} ms')
        if failures:
            raise RuntimeError('; '.join(failures))
        print('PASS: session stable under DISABLE-only command load')


def main():
    parser = argparse.ArgumentParser(
        description='Publish safe DISABLE commands while checking XRCE stability.')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--rate', type=float, default=100.0)
    args, ros_args = parser.parse_known_args()
    if not 10.0 <= args.duration <= 600.0:
        parser.error('--duration must be within 10..600 seconds')
    if not 1.0 <= args.rate <= 200.0:
        parser.error('--rate must be within 1..200 Hz')

    rclpy.init(args=ros_args)
    node = SessionLoadTest(args.rate)
    try:
        node.wait_ready(10.0)
        node.run_load(args.duration)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
