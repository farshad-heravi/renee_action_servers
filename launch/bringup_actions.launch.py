from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

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
    ]

    # start moveit (move_group is delayed ~8s inside start_moveit.launch.py)
    start_moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('renee_rbvogui_plus_moveit_config'), 'launch', 'start_moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': 'true',
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

    # Start both after move_group + RViz timer in start_moveit (8s) with margin
    delayed_arm_motion_server = TimerAction(
        period=30.0,
        actions=[arm_motion_plan_server, arm_joint_motion_plan_server],
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
            delayed_arm_motion_server,
            # container,
        ]
    )
