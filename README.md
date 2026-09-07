# renee_action_servers

`bringup_actions.launch.py` is the common entry point for simulation and the
real RB-VOGUI+ UR5e.

Simulation:

```bash
ros2 launch renee_action_servers bringup_actions.launch.py
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
