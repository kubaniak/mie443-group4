# ROS 2 Parameters for mie443_contest2

This document details the ROS 2 parameters that can be used to tweak the behavior of the `contest2` node.

## Modifying Parameters
You can modify these parameters by editing the YAML configuration file located at:
`config/params.yaml`

To run the node using this file, use the provided launch file:
```bash
ros2 launch mie443_contest2 contest2.launch.py
```

## Parameter List

### General Settings
- **`contest_time_limit`**
  - **Type:** `int`
  - **Default:** `300`
  - **Description:** The time limit (in seconds) for the contest before the node throws a warning.
  - **Bounds:** Must be `> 0`.

- **`manipulable_object`**
  - **Type:** `string`
  - **Default:** `"cup"`
  - **Description:** The YOLO class label of the main object you want to pick up and manipulate.

### Initial Behavior
- **`spin_angular_speed`**
  - **Type:** `double`
  - **Default:** `0.5`
  - **Description:** The rotational speed (rad/s) used during the initial 360-degree scan around the map.
  - **Bounds:** Should be within `[-0.5, 0.5]` rad/s based on standard TurtleBot limits.

- **`spin_total_angle`**
  - **Type:** `double`
  - **Default:** `6.28318530718` (2 * π)
  - **Description:** The total angular distance (radians) the robot must spin to complete its initial surroundings check.

### Detection and Box Navigation
- **`box_offset`**
  - **Type:** `double`
  - **Default:** `0.7`
  - **Description:** The distance (in meters) to stop away from the box center when initially navigating to a bin.
  - **Bounds:** Should be large enough to not hit the box (e.g., `> 0.3`).

- **`closer_offset`**
  - **Type:** `double`
  - **Default:** `0.3`
  - **Description:** The distance (in meters) to move closer to the box if an object is not detected at the initial `box_offset`.

- **`min_detection_confidence`**
  - **Type:** `double`
  - **Default:** `0.6`
  - **Description:** The minimum YOLO confidence score required for an object detection to be considered valid.
  - **Bounds:** `[0.0, 1.0]`.

### Scanning/Panning Behavior
- **`pan_speed`**
  - **Type:** `double`
  - **Default:** `0.08`
  - **Description:** The angular velocity (rad/s) the robot turns when panning left and right to search for an object on a box.

- **`pan_steps`**
  - **Type:** `int`
  - **Default:** `5`
  - **Description:** The number of discrete steps the robot makes in each direction while scanning for objects.

- **`pan_step_ms`**
  - **Type:** `int`
  - **Default:** `900`
  - **Description:** The duration (in milliseconds) the robot turns per step when scanning.

- **`settle_ms`**
  - **Type:** `int`
  - **Default:** `250`
  - **Description:** The duration (in milliseconds) to wait after stopping a pan step before attempting a detection, allowing the camera image to stabilize.

### AprilTag Alignment Controller
- **`align_desired_distance_x`**
  - **Type:** `double`
  - **Default:** `0.3`
  - **Description:** The target longitudinal distance (in meters, straight ahead) to maintain from the AprilTag.

- **`align_desired_distance_y`**
  - **Type:** `double`
  - **Default:** `0.0`
  - **Description:** The target lateral distance (in meters, left/right) relative to the AprilTag. Typically `0.0` to be centered.

- **`align_kP_linear`**
  - **Type:** `double`
  - **Default:** `0.5`
  - **Description:** The proportional gain for correcting linear (forward/backward) error relative to the AprilTag.

- **`align_kP_angular`**
  - **Type:** `double`
  - **Default:** `1.0`
  - **Description:** The proportional gain for correcting angular heading error to face the AprilTag directly.

- **`align_dist_tolerance`**
  - **Type:** `double`
  - **Default:** `0.05`
  - **Description:** The acceptable linear error margin (in meters) to consider alignment successful.

- **`align_angle_tolerance`**
  - **Type:** `double`
  - **Default:** `0.05`
  - **Description:** The acceptable angular error margin (in radians) to consider alignment successful.
