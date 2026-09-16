# mini_servo
ROS 2 packages for viewing and planning with a mini hobby-servo robot arm.

The stack targets the small 6-axis hobby arms often sold as a "Mini Industrial Robotic Arm" or "ABB 6 Axis Robot Mechanical Arm". Older kits use 3 large and 3 small hobby servos; newer metal kits often use 6 large servos.

It is intended for **ROS 2 Rolling** (Ubuntu 24.04) and **ROS 2 Lyrical** (Ubuntu 26.04). On macOS, use Pixi + RoboStack Lyrical (see below).

![Mini Servo Robot](/docs/mini_servo_robot.jpg "Example MiniServo Robot")

## Packages

- `mini_servo_model` — URDF/xacro, meshes, and RViz visualization
- `mini_servo_moveit` — MoveIt 2 planning config and demo launch
- `mini_servo_gazebo` — Gazebo sim with a feedforward position controller and `ros_gz` bridge
- `mini_servo_hardware` — real-robot serial hardware plugin plus the same feedforward controller
- `mini_servo_firmware` — Arduino sketch for an Adafruit PCA9685 PWM servo driver

Meshes were hand-built in Onshape and are included for gross visualization.

![Robot in RViz](/docs/robot_in_rviz.png "Mini Servo Robot in RViz")

## Build

Clone this repository into a colcon workspace `src` directory (or run `colcon` from the repo root):

```bash
cd ~/mini_servo_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

### macOS with Pixi + RoboStack

Pixi is already enough; no system ROS install is required. From this repo:

```bash
pixi install
pixi run build
pixi run visualize
```

That uses RoboStack **Lyrical** for `osx-arm64`. Other tasks: `pixi run demo`, `pixi run gazebo`, `pixi run hardware`. Gazebo on macOS is the least proven path; RViz visualization and MoveIt mock demo are the ones to try first.

## Launch

Visualize the arm with a joint-state GUI:

```bash
ros2 launch mini_servo_model visualize_robot.launch.py
```

Plan in MoveIt 2 with mock `ros2_control` hardware:

```bash
ros2 launch mini_servo_moveit demo.launch.py
```

Run the arm in Gazebo with a feedforward position controller:

```bash
ros2 launch mini_servo_gazebo sim.launch.py
```

Command joint positions (radians) after the sim is up:

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

Drive the physical arm over serial (same feedforward command topic as Gazebo):

```bash
ros2 launch mini_servo_hardware hardware.launch.py serial_port:=/dev/ttyUSB0
```

The hardware plugin talks to `mini_servo_firmware/mini_servo.ino` at 500000 baud using comma-separated degrees. The Arduino firmware currently echoes commanded angles rather than encoder feedback.

## Known issues

- Kinematics are approximate
- Joint limits may need calibration and might be flipped in direction
