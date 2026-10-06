from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _on_success_or_shutdown(actions, failure_message):
    def _handler(event, context):
        if event.returncode == 0:
            return actions

        reason = f'{failure_message} wait_for_ros exited with code {event.returncode}.'
        return [
            LogInfo(msg=reason),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    return _handler

def generate_launch_description():
    """Bring up MoveIt and its optional action servers after move_group is ready."""
    
    # server_component = ComposableNode(
    #     package='renee_action_servers',
    #     plugin='renee_action_servers::ApproachPoseActionServer',
    #     name='approach_pose_action_server',
    #     parameters=[robot_description_kinematics],
    # )

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'real_robot',
            default_value='false',
            choices=['true', 'false'],
            description='Connect MoveIt to the real UR arm (ur_type) instead of Gazebo',
        ),
        # Declared before kinematics_params_file, whose default is built from it.
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            choices=['ur5e', 'ur15'],
            description='UR arm model; in sim it must match the Gazebo spawn, in real the connected arm',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.0.101',
            description='IP address of the UR arm (required in real mode)',
        ),
        DeclareLaunchArgument(
            'reverse_ip',
            default_value='192.168.0.150',
            description='IP address of this PC as reached by the UR arm',
        ),
        DeclareLaunchArgument(
            'kinematics_params_file',
            default_value=['/renee/', LaunchConfiguration('ur_type'), '_calibration.yaml'],
            description='Absolute path to the extracted calibration YAML of the UR arm',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start a dedicated MoveIt RViz instance',
        ),
        DeclareLaunchArgument(
            'start_camera_placement',
            default_value='false',
            description='Start the /camera_placement planner action server',
        ),
        DeclareLaunchArgument(
            'use_camera_rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Open the dedicated RGB/depth RealSense RViz preview',
        ),
        DeclareLaunchArgument(
            'start_screw_pose',
            default_value='false',
            description='Start the /detect_screw action server (needs the vision module screw_pose node running)',
        ),
        DeclareLaunchArgument('realsense_serial_no', default_value="''"),
        DeclareLaunchArgument('realsense_depth_profile', default_value='848x480x30'),
        DeclareLaunchArgument('realsense_color_profile', default_value='848x480x30'),
        DeclareLaunchArgument('realsense_infra_profile', default_value='848x480x30'),
        DeclareLaunchArgument(
            'wrist_camera',
            default_value='stereolabs_zed2i',
            choices=['stereolabs_zed2i', 'realsense_d435i', 'none'],
            description='Camera type to use on the wrist (none = no camera)',
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='pointer_tester',
            choices=['none', 'pointer_tester'],
            description='Tool mounted on the real arm (real_robot:=true only)',
        ),
        DeclareLaunchArgument(
            'is_localization_enabled',
            default_value='false',
            description='Localization publishes robot_map→robot_odom; if false, a static transform is published instead',
        ),
    ]

    # start moveit (move_group is delayed ~8s inside start_moveit.launch.py)
    effective_use_sim_time = ParameterValue(
        PythonExpression([
            "'", LaunchConfiguration('real_robot'), "' != 'true' and '",
            LaunchConfiguration('use_sim_time'), "' == 'true'",
        ]),
        value_type=bool,
    )
    start_moveit_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('renee_rbvogui_plus_moveit_config'), 'launch', 'start_moveit.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('real_robot')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'wrist_camera': LaunchConfiguration('wrist_camera'),
            'is_localization_enabled': LaunchConfiguration('is_localization_enabled'),
            'ur_type': LaunchConfiguration('ur_type'),
        }.items(),
    )
    start_moveit_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('renee_rbvogui_plus_moveit_config'),
                'launch',
                'start_moveit_real.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('real_robot')),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'reverse_ip': LaunchConfiguration('reverse_ip'),
            'kinematics_params_file': LaunchConfiguration('kinematics_params_file'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'wrist_camera': LaunchConfiguration('wrist_camera'),
            'end_effector': LaunchConfiguration('end_effector'),
            'ur_type': LaunchConfiguration('ur_type'),
        }.items(),
    )

    # MoveIt-backed ArmMotionPlan server at /moveit_arm_motion_plan (matches ApproachPose default)
    arm_motion_plan_server = Node(
        package='renee_action_servers',
        executable='arm_motion_plan_server',
        name='arm_motion_plan_server',
        output='screen',
        parameters=[{
            'use_sim_time': effective_use_sim_time,
            'move_group_namespace': 'robot',
            'planning_group': 'arm',
        }],
    )

    # MoveIt-backed ArmJointMotionPlan server at /moveit_arm_joint_motion_plan
    arm_joint_motion_plan_server = Node(
        package='renee_action_servers',
        executable='arm_joint_motion_plan_server',
        name='arm_joint_motion_plan_server',
        output='screen',
        parameters=[{
            'use_sim_time': effective_use_sim_time,
            'move_group_namespace': 'robot',
            'planning_group': 'arm',
        }],
    )

    camera_placement_server = Node(
        package='renee_action_servers',
        executable='camera_placement_action_server',
        name='camera_placement_action_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('renee_action_servers'),
                'config',
                'camera_placement.yaml',
            ]),
            {'use_sim_time': effective_use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('start_camera_placement')),
    )

    camera_frames_server = Node(
        package='renee_action_servers',
        executable='capture_camera_frames_action_server',
        name='capture_camera_frames_action_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('renee_action_servers'),
                'config',
                'camera_frames_real.yaml',
            ]),
            {'use_sim_time': effective_use_sim_time},
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('real_robot'), "' == 'true' and '",
            LaunchConfiguration('wrist_camera'), "' == 'stereolabs_zed2i'",
        ])),
    )

    screw_pose_server = Node(
        package='renee_action_servers',
        executable='screw_pose_action_server',
        name='screw_pose_action_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('renee_action_servers'),
                'config',
                'screw_pose_action.yaml',
            ]),
            {'use_sim_time': effective_use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('start_screw_pose')),
    )

    wrist_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('renee_perception'),
                'launch',
                'realsense_wrist.launch.py',
            )
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_camera_rviz'),
            'serial_no': LaunchConfiguration('realsense_serial_no'),
            'depth_profile': LaunchConfiguration('realsense_depth_profile'),
            'color_profile': LaunchConfiguration('realsense_color_profile'),
            'infra_profile': LaunchConfiguration('realsense_infra_profile'),
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('real_robot'), "' == 'true' and '",
            LaunchConfiguration('wrist_camera'), "' == 'realsense_d435i'",
        ])),
    )

    # Start action servers once move_group's action interface is actually up.
    wait_for_action_server_move_group = ExecuteProcess(
        cmd=['wait_for_ros', '--timeout', '90', 'action', '/robot/move_action'],
        name='wait_for_action_server_move_group',
        output='screen',
    )
    start_arm_motion_servers_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_action_server_move_group,
            on_exit=_on_success_or_shutdown(
                [arm_motion_plan_server, arm_joint_motion_plan_server],
                'Not starting arm action servers:',
            ),
        )
    )

    # container = ComposableNodeContainer(
    #     name='renee_action_server_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[server_component],
    #     output='screen',
    # )

    # ur_external_control_monitor_node = Node(
    #     package='wrs25_arm_actions',
    #     executable='ur_external_control_monitor.py',
    #     name='ur_external_control_monitor',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('real_robot')),
    # )

    return LaunchDescription(
        declared_arguments + [
            start_moveit_sim,
            start_moveit_real,
            wrist_camera_launch,
            camera_placement_server,
            camera_frames_server,
            screw_pose_server,
            wait_for_action_server_move_group,
            start_arm_motion_servers_when_ready,
            # container,
        ]
    )
