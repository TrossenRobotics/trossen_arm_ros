# ROS 2 Packages for Trossen Robotics Arms

# Overview

This repository contains ROS 2 packages for controlling arms from Trossen Robotics.

# Installation

> [!NOTE]
> This package requires that ROS 2 Humble is already installed on your system.
> For more information on how to install ROS 2 Humble, please refer to the [official installation guide](https://docs.ros.org/en/humble/Installation.html).

1. Create a ROS 2 workspace and clone the repository:

   ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/TrossenRobotics/trossen_arm_ros.git
    ```

2. Pull in the dependencies using rosdep and vcs:

    ```bash
    cd ~/ros2_ws
    vcs import src < src/trossen_arm_ros/dependencies.repos
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace:

    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

# Usage

## Launching Trossen Arm Hardware Interface

To launch the Trossen Arm hardware interface with controllers for the arm and gripper, use the following command:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch trossen_arm_bringup trossen_arm.launch.py
```

## Launching MoveIt with Mock Hardware

To launch simulated hardware with MoveIt, use the following command:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch trossen_arm_moveit moveit.launch.py ros2_control_hardware_type:=mock_components
```

## Launching MoveIt with Real Hardware

To launch real hardware with MoveIt, use the following command:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch trossen_arm_moveit moveit.launch.py ros2_control_hardware_type:=real
```
