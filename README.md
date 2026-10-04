# renee_action_servers

`bringup_actions.launch.py` is the common entry point for simulation and the
real RB-VOGUI+ UR5e.

Internally it selects one dedicated launch:

- `start_moveit.launch.py` for simulation.
- `start_moveit_real.launch.py` for the real UR5e driver and MoveIt.

Simulation:

```bash
ros2 launch renee_action_servers bringup_actions.launch.py \
  wrist_camera:=stereolabs_zed2i \
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
  wrist_camera:=stereolabs_zed2i \
  use_rviz:=true
```

Real mode always disables simulation time, starts
`scaled_joint_trajectory_controller`, and exposes MoveIt through
`/robot/move_action`. The application-facing action servers remain:

- `/moveit_arm_motion_plan`
- `/moveit_arm_joint_motion_plan`
- `/capture_camera_frames` (real robot with `wrist_camera:=stereolabs_zed2i`: frames from the ZED on the Jetson)

Capture five RGB-D frames from the ZED (see `docs/camera_link_protocol.md`):

```bash
ros2 action send_goal /capture_camera_frames \
  renee_action_servers/action/CaptureCameraFrames \
  "{mode: rgbd, num_frames: 5}"
```

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

## Camera placement (real robot, real map)

`/camera_placement` only **plans**: for a desired pose of a frame on the arm
(the camera optical frame, or any other frame such as the pointer_tester tip)
in `robot_map`, it samples base poses around it, keeps the ones with a
collision-free MoveIt IK and a Nav2 path, and returns the best base pose and
arm joint solution. Nothing moves; the result is then executed with Nav2 and
`/moveit_arm_joint_motion_plan`.

Needs `bridge-real`, `localize-real` and `navigate-real` running (Nav2's
`/robot/compute_path_to_pose`), then:

```bash
ros2 launch renee_action_servers bringup_actions.launch.py \
  real_robot:=true robot_ip:=192.168.0.101 reverse_ip:=192.168.0.150 \
  use_rviz:=true start_camera_placement:=true
```

1. Plan: put the pointer_tester tip 1 m above the floor at (1.0, 0.5) in the
   map, its +Z (along the pointer) horizontal towards +X (the pose's
   orientation is the frame's: here +Z rotated onto +X, a 90 deg pitch).
   `camera_link` overrides the configured camera frame for this goal;
   `lock_current_base: true` would keep the base where it is and only solve the arm.

   ```bash
   ros2 action send_goal --feedback /camera_placement \
     renee_action_servers/action/CameraPlacement \
     "{camera_pose: {header: {frame_id: robot_map},
                     pose: {position: {x: 1.0, y: 0.5, z: 1.0},
                            orientation: {x: 0.0, y: 0.7071068, z: 0.0, w: 0.7071068}}},
       camera_link: robot_arm_pointer_tester_tip,
       lock_current_base: false}"
   ```

   The result has `base_pose` (robot_map) and `arm_solution` (joint names and positions).

2. Drive the base to `base_pose` (copy its position and orientation):

   ```bash
   ros2 action send_goal /robot/navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: robot_map},
              pose: {position: {x: <base_pose.x>, y: <base_pose.y>, z: 0.0},
                     orientation: {z: <base_pose.qz>, w: <base_pose.qw>}}}}"
   ```

3. Move the arm to `arm_solution` (slowly, e-stop at hand):

   ```bash
   ros2 action send_goal /moveit_arm_joint_motion_plan \
     renee_action_servers/action/ArmJointMotionPlan \
     "{joint_names: [robot_arm_shoulder_pan_joint, robot_arm_shoulder_lift_joint, robot_arm_elbow_joint,
                     robot_arm_wrist_1_joint, robot_arm_wrist_2_joint, robot_arm_wrist_3_joint],
       joint_positions: [<arm_solution.position>],
       planning_pipeline_id: pilz_industrial_motion_planner, planner_id: PTP,
       velocity_scaling: 0.1, acceleration_scaling: 0.1}"
   ```

   Check the result against the goal: `ros2 run tf2_ros tf2_echo robot_map robot_arm_pointer_tester_tip`.
