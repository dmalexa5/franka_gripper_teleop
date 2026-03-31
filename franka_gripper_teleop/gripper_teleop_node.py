#!/usr/bin/env python3
# Copyright (c) 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
from typing import List, Optional

from action_msgs.msg import GoalStatus
from franka_msgs.action import Grasp, Move
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - handled as runtime dependency problem
    serial = None

    class SerialException(Exception):
        """Fallback exception when pyserial is not importable."""


class CommandState(str, Enum):
    OPEN = 'OPEN'
    GRIP = 'GRIP'
    NONE = 'NONE'


class FrankaGripperTeleopNode(Node):

    def __init__(self) -> None:
        super().__init__('gripper_teleop_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud_rate', 19200)
        self.declare_parameter('serial_timeout_sec', 0.02)
        self.declare_parameter('command_frequency_hz', 10.0)
        self.declare_parameter('grasp_force', 10.0)
        self.declare_parameter('grasp_speed', 0.1)
        self.declare_parameter('move_speed', 0.1)
        self.declare_parameter('close_width', 0.0)
        self.declare_parameter('open_width', 0.08)
        self.declare_parameter('grasp_epsilon_inner', 0.005)
        self.declare_parameter('grasp_epsilon_outer', 0.005)
        self.declare_parameter('grasp_action_name', 'franka_gripper/grasp')
        self.declare_parameter('move_action_name', 'franka_gripper/move')
        self.declare_parameter('warn_throttle_sec', 5.0)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.serial_baud_rate = int(self.get_parameter('serial_baud_rate').value)
        self.serial_timeout_sec = float(self.get_parameter('serial_timeout_sec').value)
        self.command_frequency_hz = float(self.get_parameter('command_frequency_hz').value)
        self.grasp_force = float(self.get_parameter('grasp_force').value)
        self.grasp_speed = float(self.get_parameter('grasp_speed').value)
        self.move_speed = float(self.get_parameter('move_speed').value)
        self.close_width = float(self.get_parameter('close_width').value)
        self.open_width = float(self.get_parameter('open_width').value)
        self.grasp_epsilon_inner = float(self.get_parameter('grasp_epsilon_inner').value)
        self.grasp_epsilon_outer = float(self.get_parameter('grasp_epsilon_outer').value)
        self.grasp_action_name = str(self.get_parameter('grasp_action_name').value)
        self.move_action_name = str(self.get_parameter('move_action_name').value)
        self.warn_throttle_sec = float(self.get_parameter('warn_throttle_sec').value)

        if self.command_frequency_hz <= 0.0:
            self.get_logger().warn(
                'Parameter command_frequency_hz must be > 0.0. Using 10.0 Hz instead.'
            )
            self.command_frequency_hz = 10.0

        self._grasp_action_client = ActionClient(self, Grasp, self.grasp_action_name)
        self._move_action_client = ActionClient(self, Move, self.move_action_name)
        self._serial_connection: Optional['serial.Serial'] = None

        self._last_valid_state = CommandState.OPEN
        self._last_dispatched_state: Optional[CommandState] = None
        self._goal_futures: List[rclpy.task.Future] = []
        self._result_futures: List[rclpy.task.Future] = []

        timer_period = 1.0 / self.command_frequency_hz
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'Started with serial_port={self.serial_port}, command_frequency_hz='
            f'{self.command_frequency_hz:.2f}'
        )

    def _timer_callback(self) -> None:
        desired_state = self._read_serial_command()

        if desired_state == CommandState.NONE:
            self.get_logger().warn(
                'Serial interface unavailable. Sending OPEN command until the interface returns.',
                throttle_duration_sec=self.warn_throttle_sec,
            )

        if desired_state == self._last_dispatched_state:
            return

        if desired_state == CommandState.GRIP:
            sent = self._send_close_goal()
        else:
            sent = self._send_open_goal()

        if sent:
            self._last_dispatched_state = desired_state

    def _read_serial_command(self) -> CommandState:
        if not self._ensure_serial_connection():
            return CommandState.NONE

        assert self._serial_connection is not None
        try:
            raw_line = self._serial_connection.readline()
            while raw_line and self._serial_connection.in_waiting > 0:
                next_line = self._serial_connection.readline()
                if not next_line:
                    break
                raw_line = next_line
        except SerialException as exc:
            self.get_logger().warn(
                f'Error while reading serial input: {exc}. Falling back to NONE mode.',
                throttle_duration_sec=self.warn_throttle_sec,
            )
            self._close_serial_connection()
            return CommandState.NONE

        if not raw_line:
            return self._last_valid_state

        token = raw_line.decode('utf-8', errors='ignore').strip().upper()
        if token == 'OPEN':
            self._last_valid_state = CommandState.OPEN
        elif token == 'GRIP':
            self._last_valid_state = CommandState.GRIP
        elif token:
            self.get_logger().warn(
                f"Unknown serial token '{token}'. Keeping '{self._last_valid_state.value}'.",
                throttle_duration_sec=self.warn_throttle_sec,
            )

        return self._last_valid_state

    def _ensure_serial_connection(self) -> bool:
        if serial is None:
            self.get_logger().warn(
                "python3-serial is not available. Falling back to NONE mode.",
                throttle_duration_sec=self.warn_throttle_sec,
            )
            return False

        if self._serial_connection is not None and self._serial_connection.is_open:
            return True

        try:
            self._serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baud_rate,
                timeout=self.serial_timeout_sec,
            )
            self.get_logger().info(
                f'Connected to serial interface {self.serial_port} at '
                f'{self.serial_baud_rate} baud.'
            )
            return True
        except SerialException as exc:
            self._serial_connection = None
            self.get_logger().warn(
                f'Failed to open serial interface {self.serial_port}: {exc}.',
                throttle_duration_sec=self.warn_throttle_sec,
            )
            return False

    def _close_serial_connection(self) -> None:
        if self._serial_connection is None:
            return

        try:
            if self._serial_connection.is_open:
                self._serial_connection.close()
        except SerialException:
            pass
        self._serial_connection = None

    def _send_close_goal(self) -> bool:
        if not self._grasp_action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                f"Action server '{self.grasp_action_name}' is unavailable.",
                throttle_duration_sec=self.warn_throttle_sec,
            )
            return False

        goal = Grasp.Goal()
        goal.width = self.close_width
        goal.speed = self.grasp_speed
        goal.force = self.grasp_force
        goal.epsilon.inner = self.grasp_epsilon_inner
        goal.epsilon.outer = self.grasp_epsilon_outer

        goal_future = self._grasp_action_client.send_goal_async(goal)
        goal_future.add_done_callback(self._goal_response_callback)
        self._goal_futures.append(goal_future)

        self.get_logger().info('Sent GRIP gripper goal.')
        return True

    def _send_open_goal(self) -> bool:
        if not self._move_action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                f"Action server '{self.move_action_name}' is unavailable.",
                throttle_duration_sec=self.warn_throttle_sec,
            )
            return False

        goal = Move.Goal()
        goal.width = self.open_width
        goal.speed = self.move_speed

        goal_future = self._move_action_client.send_goal_async(goal)
        goal_future.add_done_callback(self._goal_response_callback)
        self._goal_futures.append(goal_future)

        self.get_logger().info('Sent OPEN gripper goal.')
        return True

    def _goal_response_callback(self, future: rclpy.task.Future) -> None:
        self._remove_future(self._goal_futures, future)

        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - depends on action transport errors
            self.get_logger().error(f'Failed to send action goal: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Action goal was rejected.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        self._result_futures.append(result_future)

    def _result_callback(self, future: rclpy.task.Future) -> None:
        self._remove_future(self._result_futures, future)

        try:
            wrapped_result = future.result()
        except Exception as exc:  # pragma: no cover - depends on action transport errors
            self.get_logger().error(f'Failed to get action result: {exc}')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'Action finished with status={wrapped_result.status}.',
                throttle_duration_sec=self.warn_throttle_sec,
            )
            return

        result = wrapped_result.result
        if not result.success:
            self.get_logger().warn(
                f"Action reported failure: '{result.error}'.",
                throttle_duration_sec=self.warn_throttle_sec,
            )

    @staticmethod
    def _remove_future(storage: List[rclpy.task.Future], future: rclpy.task.Future) -> None:
        if future in storage:
            storage.remove(future)

    def destroy_node(self) -> bool:
        self._close_serial_connection()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = FrankaGripperTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
