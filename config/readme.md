# Configuration Parameters Documentation

This document provides a brief explanation of the JSON configuration parameters used in the system, divided into three sections:

1. **Device Settings**
2. **Task Configuration**
3. **Data Processing Configuration**

---

## Table of Contents

1. [Device Settings](#device-settings)
2. [Task Configuration](#task-configuration)
3. [Data Processing Configuration](#data-processing-configuration)

---

## Device Settings

Defines the robot type, data directory, and computational device.

```json
"device_settings": {
    "robot_type": "XARM6",
    "data_dir": "./",
    "device": "cpu"
}
```

- **robot_type** (`string`): Type of the robot.
- **data_dir** (`string`): Path for data storage.
- **device** (`string`): Computational device.

---

## Task Configuration

Sets the task duration, state and action dimensions, camera settings, and ROS topics.

```json
"task_config": {
    "episode_len": 180,
    "state_dim": 7,
    "action_dim": 7,
    "cam_width": 1920,
    "cam_height": 1080,
    "camera_names": ["front"],
    "camera_port": 0,
    "ros": {
        "video_topic": "/usb_cam/image_raw",
        "trajectory_topic": "/camera/odom/sample",
        "queue_size": 1000
    }
}
```

- **episode_len** (`integer`): Length of each task episode, please adjust according to the task duration.
- **state_dim** (`integer`): Dimension of the state space.
- **action_dim** (`integer`): Dimension of the action space.
- **cam_width** (`integer`): Camera width in pixels.
- **cam_height** (`integer`): Camera height in pixels.
- **camera_names** (`array`): Names of the cameras.
- **camera_port** (`integer`): Camera port number.

### ROS Configuration

- **video_topic** (`string`): ROS topic for video data.
- **trajectory_topic** (`string`): ROS topic for trajectory data.
- **queue_size** (`integer`): Size of the ROS message queue.

---

## Data Processing Configuration

Manages data processing paths, markers, robot position, and initial joint states. 
We have already performed coordinate transformation in the code. The corresponding relationship diagram of coordinate transformation is shown below:


<figure align="center">
  <img src="../docs/2.png" width="400" />
  <img src="../docs/3.jpg" width="400" />
  <figcaption>Coordinate system of T265 in rviz</figcaption>
</figure>
<figure align="center">
  <img src="../docs/1.png" width="400" />
  <figcaption>Coordinate system of Xarm6</figcaption>
</figure>


The following configuration is based on the coordinate system conversion shown in the figure.


```json
"data_process_config": {
    "marker_id_0":0,
    "marker_id_1":1,
    "input_dir": "./dataset/test",
    "output_joint_dir": "./dataset/test_joint_with_gripper",
    "output_tcp_dir": "./dataset/test_tcp_with_gripper",
    "dp_train_data_dir": "./dataset/dp_train_data.zarr.zip",
    "dp_data_res": "224, 224",
    "compression_level": 99,
    "urdf_path": "./assets/xarm6_robot.urdf",
    "aruco_dict": "DICT_4X4_50",
    "base_position": {
        "x": 0.49643,
        "y": -0.00588,
        "z": 0.35984
    },
    "base_orientation": {
        "roll": 179.94725,
        "pitch": -89.999981,
        "yaw": 0.0
    },
    "offset": {
        "x": 0.14565,
        "z": 0.1586
    },
    "distances": {
        "marker_max": 566.0,
        "marker_min": 140.0,
        "gripper_max": 850.0,
        "gripper_min": 0,
        "flange_to_tcp": 0.25
    },
    "start_qpos": [0, 0, -0.0279, -0.4747, -0.0384, -0.0227, -1.0577, 0.0035]
}
```

- **marker_id_0 / marker_id_1** (`integer`): IDs for ArUco markers.
- **input_dir** (`string`): Path for input data.
- **output_joint_dir / output_tcp_dir** (`string`): Paths for output data.
- **dp_train_data_dir** (`string`): Path to training data archive.
- **dp_data_res** (`string`): Image resolution for data processing.
- **compression_level** (`integer`): Compression level for data storage.
- **urdf_path** (`string`): Path to the URDF file.
- **aruco_dict** (`string`): Type of ArUco dictionary.

### Position & Orientation

- **base_position**: Initial Tool Center Point position of Robot arm (there is Xarm6).
- **base_orientation**: Initial Tool Center Point orientation of Robot arm (there is Xarm6).
- **offset**: Positional offsets of the RealSense T265 relative to the Tool Center Point on Handheld Device.

### Distance Parameters

- **marker_max / marker_min** (`float`): Marker detection distance range.
- **gripper_max / gripper_min** (`float`): Gripper open/close range (Refers to the value transmitted in the gripper control command).
- **flange_to_tcp** (`float`): Distance from flange to Tool Center Point.

### Initial Joint Positions

- **start_qpos** (`array`): Initial positions for each joint of Robot arm (there is Xarm6).

---

## Summary

This configuration file is essential for setting up the robot's hardware, task parameters, and data processing workflows. Adjust these parameters according to your specific needs to ensure paths and values correctly match your system setup.

For further assistance or questions, please refer to the system documentation or contact the support team.
