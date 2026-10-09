#!/usr/bin/env python3
"""Safely validate the ROS 2 -> micro-ROS -> STM32 -> joint1 control path."""

import argparse
import math
import time

import rclpy
from microros_stm32h723_bridge.msg import HardwareStatus, JointCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

COMMAND_LOSS_LATCHED_FAULT = 11


class Joint1IntegrationTest(Node):
    def __init__(self, kp, kd):
        super().__init__('joint1_microros_integration_test')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.publisher = self.create_publisher(
            JointCommand, '/stm32/joint_commands', qos)
        self.create_subscription(
            JointState, '/stm32/joint_states', self._on_joint, qos)
        self.create_subscription(
            HardwareStatus, '/stm32/status', self._on_status, qos)
        self.kp = kp
        self.kd = kd
        self.joint = None
        self.status = None
        self.sequence = 1
        self.positions = []
        self.lockout_confirmed = False

    def _on_joint(self, message):
        if message.position and message.velocity and message.effort:
            self.joint = message
            self.positions.append(float(message.position[0]))

    def _on_status(self, message):
        self.status = message

    def wait_ready(self, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.publisher.get_subscription_count() == 1 and
                    self.joint is not None and self.status is not None):
                self.sequence = int(self.status.last_command_sequence) + 1
                return
        raise RuntimeError('STM32 subscriber or feedback/status unavailable')

    def _command(self, mode, position, kp=None, kd=None):
        message = JointCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.sequence = self.sequence
        self.sequence += 1
        message.mode = mode
        message.position = [position] + [0.0] * 6
        message.velocity = [0.0] * 7
        message.kp = [self.kp if kp is None else kp] + [0.0] * 6
        message.kd = [self.kd if kd is None else kd] + [0.0] * 6
        message.effort = [0.0] * 7
        return message

    def _check_feedback(self, initial_position):
        if self.status is None or int(self.status.fault_code) != 0:
            fault = None if self.status is None else self.status.fault_code
            raise RuntimeError(f'STM32 fault={fault}')
        if self.joint is None:
            raise RuntimeError('joint feedback disappeared')
        position = float(self.joint.position[0])
        velocity = float(self.joint.velocity[0])
        effort = float(self.joint.effort[0])
        if not all(math.isfinite(value)
                   for value in (position, velocity, effort)):
            raise RuntimeError('non-finite feedback')
        if (abs(position - initial_position) > 0.20 or
                abs(velocity) > 0.80 or abs(effort) > 1.0):
            raise RuntimeError(
                f'safety limit: p={position:.6f}, v={velocity:.6f}, '
                f'effort={effort:.6f}')

    def stream(self, name, initial_position, start, end, duration):
        print(f'PHASE {name}: {start:.6f} -> {end:.6f}', flush=True)
        phase_positions = []
        begin = time.monotonic()
        deadline = begin + duration
        while time.monotonic() < deadline:
            progress = min(1.0, (time.monotonic() - begin) / duration)
            target = start + (end - start) * progress
            self.publisher.publish(
                self._command(JointCommand.MODE_TRACK, target))
            cycle_end = time.monotonic() + 0.05
            while time.monotonic() < cycle_end:
                rclpy.spin_once(self, timeout_sec=0.01)
            self._check_feedback(initial_position)
            phase_positions.append(float(self.joint.position[0]))
        print(f'PHASE_RESULT {name}: min={min(phase_positions):.6f}, '
              f'max={max(phase_positions):.6f}, '
              f'end={phase_positions[-1]:.6f}', flush=True)
        return phase_positions

    def hold_track(self, name, initial_position, target, duration):
        return self.stream(
            name, initial_position, target, target, duration)

    def disable(self):
        for _ in range(10):
            self.publisher.publish(
                self._command(JointCommand.MODE_DISABLE, 0.0, 0.0, 0.0))
            rclpy.spin_once(self, timeout_sec=0.05)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.status is not None and int(self.status.mode) == 3:
                print('DISABLE confirmed', flush=True)
                return
        print('WARNING: DISABLE sent but status confirmation timed out', flush=True)

    def verify_timeout_lockout(self, zero_position, rejected_target):
        accepted_sequence = int(self.status.last_command_sequence)
        print('PHASE TIMEOUT_LOCKOUT: stop commands for 0.80 s', flush=True)
        deadline = time.monotonic() + 0.80
        observed_positions = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.joint is not None:
                observed_positions.append(float(self.joint.position[0]))

        if self.status is None:
            raise RuntimeError('status disappeared during timeout test')
        if int(self.status.mode) != JointCommand.MODE_HOLD:
            raise RuntimeError(
                f'timeout did not hold: mode={self.status.mode}')
        if int(self.status.fault_code) != COMMAND_LOSS_LATCHED_FAULT:
            raise RuntimeError(
                f'timeout did not latch fault: fault={self.status.fault_code}')
        if (not observed_positions or
                max(abs(value - zero_position) for value in observed_positions) >
                0.05):
            raise RuntimeError('joint moved outside 0.05 rad during timeout HOLD')

        for mode, position in (
                (JointCommand.MODE_TRACK, rejected_target),
                (JointCommand.MODE_DISABLE, 0.0)):
            for _ in range(3):
                self.publisher.publish(self._command(mode, position))
                rclpy.spin_once(self, timeout_sec=0.05)

        if (int(self.status.mode) != JointCommand.MODE_HOLD or
                int(self.status.fault_code) != COMMAND_LOSS_LATCHED_FAULT or
                int(self.status.last_command_sequence) != accepted_sequence):
            raise RuntimeError(
                'latched controller accepted a recovery command: '
                f'mode={self.status.mode}, fault={self.status.fault_code}, '
                f'sequence={self.status.last_command_sequence}, '
                f'expected_sequence={accepted_sequence}')
        self.lockout_confirmed = True
        print('TIMEOUT_LOCKOUT confirmed: HOLD, fault=11, commands rejected; '
              'STM32 restart required', flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run a guarded joint1 micro-ROS position round trip.')
    parser.add_argument('--step', type=float, default=0.10,
                        help='relative move in radians (default: 0.10)')
    parser.add_argument('--kp', type=float, default=8.0)
    parser.add_argument('--kd', type=float, default=2.0)
    parser.add_argument('--ramp-duration', type=float, default=2.0,
                        help='seconds per ramp (default: 2.0)')
    parser.add_argument('--verify-timeout-lockout', action='store_true',
                        help='at zero, verify the 500 ms restart-only HOLD latch')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.step) or not 0.02 <= abs(args.step) <= 0.10:
        parser.error('--step must be finite and 0.02..0.10 rad in magnitude')
    if not math.isfinite(args.kp) or not 0.0 <= args.kp <= 10.0:
        parser.error('--kp must be finite and within 0..10')
    if not math.isfinite(args.kd) or not 0.0 <= args.kd <= 4.0:
        parser.error('--kd must be finite and within 0..4')
    if (not math.isfinite(args.ramp_duration) or
            not 1.0 <= args.ramp_duration <= 10.0):
        parser.error('--ramp-duration must be within 1..10 seconds')
    return args, ros_args


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = Joint1IntegrationTest(args.kp, args.kd)
    try:
        node.wait_ready()
        if int(node.status.fault_code) != 0 or int(node.status.mode) != 3:
            raise RuntimeError(
                f'unsafe preflight: mode={node.status.mode}, '
                f'fault={node.status.fault_code}')
        initial = float(node.joint.position[0])
        target = initial + args.step
        if abs(target) > 0.35:
            raise RuntimeError(
                f'target outside firmware limit: {target:.6f}')
        print(f'PREFLIGHT p0={initial:.6f}, target={target:.6f}, '
              f'v0={node.joint.velocity[0]:.6f}, '
              f'effort0={node.joint.effort[0]:.6f}, seq={node.sequence}',
              flush=True)

        node.hold_track('ENABLE_AND_HOLD', initial, initial, 1.0)
        forward = node.stream(
            'TRACK_FORWARD', initial, initial, target, args.ramp_duration)
        target_hold = node.hold_track(
            'HOLD_TARGET', initial, target, 1.0)
        node.stream(
            'TRACK_RETURN', initial, target, initial, args.ramp_duration)
        final = node.hold_track('HOLD_ZERO', initial, initial, 1.0)

        observed = forward + target_hold
        displacement = ((max(observed) - initial) if args.step > 0.0
                        else (initial - min(observed)))
        final_error = abs(final[-1] - initial)
        required_motion = abs(args.step) * 0.70
        if displacement < required_motion:
            raise RuntimeError(
                f'insufficient measured motion: {displacement:.6f} rad '
                f'(required {required_motion:.6f})')
        if final_error > 0.05:
            raise RuntimeError(
                f'return error too large: {final_error:.6f} rad')
        print(f'RESULT displacement={displacement:.6f}, '
              f'final_error={final_error:.6f}, '
              f'fault={node.status.fault_code}, '
              f'samples={len(node.positions)}', flush=True)
        if args.verify_timeout_lockout:
            if abs(initial) > 0.15:
                raise RuntimeError(
                    f'timeout test requires a safe zero approach; '
                    f'p0={initial:.6f}')
            zero_ramp = node.stream(
                'TRACK_TO_ZERO', initial, initial, 0.0,
                args.ramp_duration)
            zero_hold = node.hold_track(
                'HOLD_ZERO_ABSOLUTE', initial, 0.0, 1.0)
            zero_error = abs(zero_hold[-1])
            if zero_error > 0.03:
                raise RuntimeError(
                    f'absolute zero error too large: {zero_error:.6f} rad')
            print(f'ZERO_CONFIRMED p={zero_hold[-1]:.6f}, '
                  f'ramp_samples={len(zero_ramp)}', flush=True)
            node.verify_timeout_lockout(0.0, target)
    finally:
        try:
            if not node.lockout_confirmed:
                node.disable()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
