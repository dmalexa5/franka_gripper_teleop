# franka_gripper_teleop

`franka_gripper_teleop` is a ROS 2 Python package that teleoperates a Franka gripper from a serial device. The node reads line-based commands from a serial port and forwards them to the Franka gripper action servers.

## Features

- Reads serial commands from a configurable device such as and arduino + button on `/dev/ttyACM0`
- Sends `OPEN` requests through `franka_gripper/move`
- Sends `CLOSE` requests through `franka_gripper/grasp`
- Ships with a launch file and a default parameter YAML

## Serial Commands

The node expects newline-terminated ASCII tokens on the configured serial port:

- `OPEN`
- `CLOSE` or `CLOSED` (equivalent)

If no new serial data is available, the node keeps the last valid command. If the serial interface is unavailable, the node falls back to sending `OPEN`.

## Dependencies

This package depends on:

- ROS 2 with `rclpy`
- `franka_msgs`
- `python3-serial`

## Build

From your ROS 2 workspace:

```bash
colcon build --packages-select franka_gripper_teleop
source install/setup.bash
```

## Run

Launch the teleoperation node with the default configuration:

```bash
ros2 launch franka_gripper_teleop gripper_teleop.launch.py
```

Override the parameter file or namespace if needed:

```bash
ros2 launch franka_gripper_teleop gripper_teleop.launch.py \
  parameters_file:=/path/to/gripper_teleop.yaml \
  namespace:=robot1
```

You can also run the node directly:

```bash
ros2 run franka_gripper_teleop gripper_teleop_node \
  --ros-args --params-file config/gripper_teleop.yaml
```

## Parameters

The default parameters are defined in [`config/gripper_teleop.yaml`](config/gripper_teleop.yaml).

Common parameters:

- `serial_port`: serial device path
- `serial_baud_rate`: baud rate for the serial connection
- `serial_timeout_sec`: serial read timeout
- `command_frequency_hz`: polling frequency for serial input
- `grasp_force`: force used for close commands
- `grasp_speed`: speed used for close commands
- `move_speed`: speed used for open commands
- `close_width`: target width for grasp actions
- `open_width`: target width for move actions
- `grasp_action_name`: grasp action topic name
- `move_action_name`: move action topic name

## Files

- [`launch/gripper_teleop.launch.py`](launch/gripper_teleop.launch.py) starts the node with a parameter file and optional namespace.
- [`franka_gripper_teleop/gripper_teleop_node.py`](franka_gripper_teleop/gripper_teleop_node.py) contains the teleoperation node.
- [`config/gripper_teleop.yaml`](config/gripper_teleop.yaml) contains the default runtime parameters.
