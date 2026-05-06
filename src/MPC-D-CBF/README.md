# MPC-D-CBF: Dynamic Control Barrier Function-based Model Predictive Control to Safety-Critical Obstacle-Avoidance of Mobile Robot

arXiv: https://arxiv.org/abs/2209.08539

bilibili: https: https://www.bilibili.com/video/BV1fN4y1N7pD/?vd_source=e11d8557ce1350ea4930d15280abb7e2

Github: https://github.com/jianzhuozhuTHU/MPC-D-CBF

YouTube: https://youtu.be/U3X6vqKTxRw

## Workspace Layout

This repository contains ROS packages and is intended to live under a catkin
workspace as:

```text
~/catkin_ws/
  src/
    MPC-D-CBF/
      global_path_publisher/
      jackal/
      local_map/
      local_planner/
      obs_param/
      scene/
```

If you want to bootstrap a workspace quickly, use:

```bash
bash tools/bootstrap_catkin_ws.sh
```

## Dependencies

This project targets ROS Noetic on Ubuntu 20.04.

```bash
sudo apt update
sudo apt install \
  ros-noetic-grid-map \
  ros-noetic-grid-map-rviz-plugin \
  ros-noetic-velodyne-description \
  ros-noetic-velodyne-gazebo-plugins \
  ros-noetic-lms1xx \
  ros-noetic-sick-tim \
  libcgal-dev

python3 -m pip install casadi
```

## Launch Options

### Debug-Friendly Two-Step Launch

Terminal 1 starts the environment together with Gazebo GUI and RViz, but does
not start the planner:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scene env_visualization.launch
```

If you want the dedicated side-static scene used for the final dynamic obstacle
avoidance experiment, use:

```bash
roslaunch scene env_visualization_side_static.launch
```

If you want an L-shaped occlusion scene with one hidden static obstacle and a
handcrafted static global path into the blind pocket, use:

```bash
roslaunch scene env_visualization_blind_wall_static.launch
```

Terminal 2 starts only the planner and controller:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scene planner_only.launch
```

This mode is convenient when you are iterating on `local_map`, obstacle motion,
or the simulation environment and want visualization before enabling planning.

## Planner Parameters

The default planner is now:

```text
local_planner/scripts/my_local_planner.py
```

`scene/planner_only.launch` loads planner parameters from:

```text
local_planner/config/my_planner_params.yaml
```

The algorithm implementation still lives in `my_local_planner.py`, while the
main tunable parameters are exposed as ROS params, for example:

- `my_local_planner/horizon`
- `my_local_planner/dt`
- `my_local_planner/gamma_k`
- `my_local_planner/v_max` / `my_local_planner/v_min` / `my_local_planner/omega_max`

## Documentation

- Workspace and run guide: `docs/WORKSPACE_GUIDE.md`
- Algorithm and package notes: `docs/ALGORITHM_NOTES.md`
