# renee_action_servers

`bringup_actions.launch.py` is the common entry point for simulation and the
real RB-VOGUI+ UR5e.

Internally it selects one dedicated launch:

- `start_moveit.launch.py` for simulation.
- `start_moveit_real.launch.py` for the real UR5e driver and MoveIt.

Simulation:

```bash
ros2 launch renee_action_servers bringup_actions.launch.py \
  wrist_camera:=realsense_d435i \
  start_capture_rgbd:=true \
  sim_depth_width:=848 sim_depth_height:=480 sim_depth_rate:=30
```

Real robot (start the External Control program manually on the teach pendant):

```bash
ros2 launch renee_action_servers bringup_actions.launch.py \
  real_robot:=true \
  robot_ip:=192.168.1.10 \
  reverse_ip:=192.168.1.20 \
  kinematics_params_file:=/absolute/path/ur5e_calibration.yaml \
  is_localization_enabled:=true \
  wrist_camera:=realsense_d435i \
  start_capture_rgbd:=true \
  realsense_depth_profile:=848x480x30 \
  use_rviz:=true
```

Real mode always disables simulation time, starts
`scaled_joint_trajectory_controller`, and exposes MoveIt through
`/robot/move_action`. The application-facing action servers remain:

- `/moveit_arm_motion_plan`
- `/moveit_arm_joint_motion_plan`
- `/capture_rgbd`

Capture a five-frame station (use `frame_count: 0` for the configured default
of 30):

```bash
ros2 action send_goal /capture_rgbd \
  renee_action_servers/action/CaptureRGBD \
  "{waypoint_id: station_001, session_dir: /tmp/renee_scan_session, frame_count: 5}"

To start only the real driver and MoveIt, without the application action
servers, use the dedicated launch directly:

```bash
ros2 launch renee_rbvogui_plus_moveit_config start_moveit_real.launch.py \
  robot_ip:=192.168.1.10 \
  reverse_ip:=192.168.1.20 \
  kinematics_params_file:=/absolute/path/ur5e_calibration.yaml \
  is_localization_enabled:=true \
  wrist_camera:=none \
  use_rviz:=true
```
