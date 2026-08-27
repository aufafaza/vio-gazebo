# UAV Sim Package

## Description

this package provides worlds, models, and launch file for SAFMC 2026 d1 and d2 gazebo and sitl simulation.

## Setup

1. build the package
   `colcon build --packages-select uav_sim`
2. source
   `source install/local_setup.zsh` or `source install/local_setup.bash`

## Usage

launch the launch files

- if you want to load d1 arena only (without SITL)
  `ros2 launch uav_sim gz_ardu_d1.launch.xml`

- if you want to load d2 arena only (without SITL)
  `ros2 launch uav_sim gz_ardu_d2.launch.xml`

- if you want to load d2 arena (default) with SITL
  `ros2 launch uav_sim uav_sim.launch.xml`

| Parameters  | Values                      | Description                        |
| ----------- | --------------------------- | ---------------------------------- |
| mode        | `d1` or `d2` (default)      | arena mode in `uav_sim.launch.xml` |
| use_map     | `true` or `false` (default) | use mavproxy map?                  |
| use_console | `true` or `false` (default) | use mavproxy console?              |
