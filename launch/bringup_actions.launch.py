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
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml
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
    """Bring up MoveIt, then the ArmMotionPlan action server (after move_group is up)."""
    
    # Get path to kinematics configuration
    moveit_config_pkg = get_package_share_directory('renee_rbvogui_plus_moveit_config')
    kinematics_yaml_path = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    
    # Load kinematics parameters from YAML file
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_config = yaml.safe_load(file)
    
    # Prepare parameters with proper namespace
    robot_description_kinematics = {'robot_description_kinematics': kinematics_config}
    
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
            'use_rviz',
            default_value='true',
            description='Start MoveIt RViz',
        ),
    ]

    # start moveit (move_group is delayed ~8s inside start_moveit.launch.py)
    start_moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('renee_rbvogui_plus_moveit_config'), 'launch', 'start_moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    # MoveIt-backed ArmMotionPlan server at /moveit_arm_motion_plan (matches ApproachPose default)
    arm_motion_plan_server = Node(
        package='renee_action_servers',
        executable='arm_motion_plan_server',
        name='arm_motion_plan_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'move_group_namespace': 'robot',
            'planning_group': 'arm',
        }],
    )

    # Start both action servers once move_group's action interface is actually up
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
            start_moveit_node,
            wait_for_action_server_move_group,
            start_arm_motion_servers_when_ready,
            # container,
        ]
    )
