# 🤖 Franka Panda Autonomous Pick-and-Place (ROS / MoveIt! — Gazebo)

This repository implements a robust **Pick-and-Place** pipeline for the 7-DoF Franka Emika Panda robot, utilizing 3D vision for object detection based on **Color** and **ArUco Markers**.

<p align="center" width="100%">
    <img src="assets/pick_and_place_overview.png" width="600" alt="Franka Panda in Gazebo performing a pick-and-place task">
</p>

## Key Features

The system is engineered with a modular ROS architecture, demonstrating advanced skills in software integration and motion planning.

* **Dual Perception:** Object localization via:
    * **Color Segmentation (HSV):** Detection of Red, Green, and Blue cubes.
    * **ArUco Markers:** Robust pose estimation of complex objects.
* **3D Pose Estimation:** Ray-casting projection onto the table plane, integrated with ROS TF transforms.
* **MoveIt! Controller:** Collision-free motion planning for a complete Pick-and-Place sequence, fully configured with external YAML parameters (offsets, planning time, velocity scaling).
* **State Machine:** Centralized control (`pick_and_place_core.py`) that subscribes to the shared `/cube_pose_stamped` topic and orchestrates the full sequence.

<p align="center" width="100%">
    <img src="assets/architecture_diagram.png" width="600" alt="ROS Architecture Diagram">
</p>

## Prerequisites

The project was developed and tested with ROS Noetic on Ubuntu 20.04 LTS.

Install standard ROS dependencies and the required Franka packages:
* ROS Noetic / Melodic
* Gazebo 11
* MoveIt!
* `panda_moveit_config` (or similar Franka MoveIt package)

The Python dependencies (e.g., `cv_bridge`, `image_geometry`, `tf`, `moveit_commander`) are specified in `package.xml`.

## Usage

### 1. Build and Setup

```bash
# Clone the repository into your catkin workspace's src folder
cd ~/catkin_ws/src
git clone [https://github.com/NazarioPizzicoli/Pick-and-place---Franka-Emika-Panda.git](https://github.com/NazarioPizzicoli/Pick-and-place---Franka-Emika-Panda.git) franka_lab_dev
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash

# 🤖 Franka Emika Panda Autonomous Pick-and-Place (ROS / MoveIt! — Gazebo Simulation)

Project overview
----------------
This repository implements a robust Pick-and-Place pipeline for the 7-DoF Franka Emika Panda robot in simulation (Gazebo), using ROS and MoveIt!. The system integrates 3D perception, motion planning, and a state machine to perform autonomous pick-and-place tasks in cluttered scenes.

The project demonstrates advanced skills in robotic software integration, motion planning, and 3D computer vision suitable for research and industry roles.

Key technical highlights
------------------------
- Platform: Franka Emika Panda (7 DoF) simulated with Gazebo.
- Framework: ROS (Melodic / Noetic) + MoveIt! for inverse kinematics and collision-free motion planning.
- Architecture: Modular ROS nodes (Perception, Planner/Controller, State Machine) for maintainability and extensibility.
- Perception:
  - Object detection via Color Thresholding (HSV using OpenCV).
  - Marker detection via ArUco markers (OpenCV).
  - 3D pose estimation using ray-casting and ROS TF transforms.
- Controller: MoveIt! interface controlling `panda_arm` and gripper (`panda_hand`).
- Configuration: All critical parameters (pose offsets, target poses, HSV ranges) are externalized to YAML files for easy calibration.

Repository structure
--------------------
- package.xml, CMakeLists.txt — ROS package metadata and build configuration.
- launch/ — ROS launch files for starting the simulation, MoveIt!, and the node stack.
- config/ — YAML parameter files (HSV ranges, pose offsets, node parameters).
- scripts/ — Python ROS nodes and utilities (perception, motion controller, state machine).
- README.md — Project documentation (this file).
- LICENSE — License for the project.

Node-level architecture (conceptual)
------------------------------------
- perception_color (colore.py): detects objects using HSV color thresholds, publishes PoseStamped on `/cube_pose_stamped`.
- perception_aruco (marker.py): detects ArUco markers and publishes their poses.
- motion_controller (movimento.py): MoveIt! wrapper to perform pick / place trajectories and gripper commands.
- state_machine (state_machine.py): orchestrates the operation, subscribes to perception topics, and calls motion_controller actions to execute pick-and-place.

Quick start (recommended)
-------------------------
Prerequisites
- Ubuntu (matching the ROS distro: 18.04 for Melodic, 20.04 for Noetic)
- ROS Melodic or Noetic
- Gazebo (matching your ROS distro)
- MoveIt! and a compatible Franka MoveIt configuration (e.g., `panda_moveit_config`)

Example steps
1. Clone the workspace:
```bash
git clone https://github.com/<your_username>/franka_pick_and_place_ws.git
cd franka_pick_and_place_ws/src
# Add this repository
git clone https://github.com/NazarioPizzicoli/Pick-and-place---Franka-Emika-Panda.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

2. Launch simulation + MoveIt! + stack (example)
```bash
# Launch the robot in Gazebo with MoveIt! and the package nodes
roslaunch franka_pick_and_place demo_simulation.launch
```
(Replace with the actual launch filename in `launch/`.)

3. Run the pick-and-place demo
- The state machine will wait for perception messages (`/cube_pose_stamped`). When a pose appears, it plans and executes a pick followed by a place.

Configuration and calibration
-----------------------------
- All calibration values are stored in YAML files in `config/`. Typical parameters:
  - HSV color thresholds for the target object
  - Pose offsets for gripper approach and grasp
  - Planner parameters (velocity/acceleration scaling)
- Provide a `calibrate_hsv.py` visualization tool (or use `rqt_image_view`) to tune HSV params.

Results and evaluation
----------------------
- Include (or add) a short demo GIF and links to recorded video; recruiters and interviewers like seeing tangible outcomes.
- Recommended metrics to add: success rate across N runs, average pick time, failure cases (collisions, mis-detections).

Why this project is relevant to recruiters (CV bullets)
-------------------------------------------------------
- Designed and implemented a full-stack robotic solution integrating perception, planning and control for a 7-DoF manipulator.
- Implemented robust 3D object localization using color segmentation and ArUco markers; integrated with TF and MoveIt!.
- Built modular ROS nodes for perception, planning and state sequencing to increase testability and reusability.
- Tuned motion planner parameters for trajectory smoothness and collision avoidance in simulation.
- Demonstrated practical experience with ROS, Gazebo, MoveIt!, OpenCV and modern robotics software practices.

Roadmap & suggested improvements
-------------------------------
(These items will strengthen the project and its appeal)
- Add a Dockerfile / reproducible CI environment to reproduce the workspace easily.
- Add GitHub Actions: build the package, run linters (flake8/black), and optionally run simulation smoke-tests.
- Write unit and integration tests for the Python nodes (rosbag-based integration tests for perception).
- Add demo assets (GIFs, short videos) and a `results/` section with performance statistics.
- Rename Italian file names to English for consistency and wider audience comprehension.
- Add an architecture diagram (SVG/PNG) and sequence diagram for the state machine.
- Provide step-by-step troubleshooting and hardware safety notes if porting to real robot.

License
-------
This project includes a LICENSE file. If you prefer a permissive license for recruiters/companies, consider MIT or Apache 2.0; if you prefer copyleft, GPL is okay — choose intentionally and document the decision.

Contact
-------
- Author: Nazario Pizzicoli
- GitHub: https://github.com/NazarioPizzicoli
- Email: (add your preferred contact email here)

Acknowledgements
----------------
This project builds on open-source ROS and MoveIt! libraries, and uses OpenCV for perception tasks.
