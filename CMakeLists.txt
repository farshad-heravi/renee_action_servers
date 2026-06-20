cmake_minimum_required(VERSION 3.8)
project(renee_action_servers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmMotionPlan.action"
  "action/ArmJointMotionPlan.action"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(arm_motion_plan_server src/arm_motion_plan_server.cpp)
ament_target_dependencies(arm_motion_plan_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  tf2_ros
)
target_link_libraries(arm_motion_plan_server "${cpp_typesupport_target}")

add_executable(arm_joint_motion_plan_server src/arm_joint_motion_plan_server.cpp)
ament_target_dependencies(arm_joint_motion_plan_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  tf2_ros
)
target_link_libraries(arm_joint_motion_plan_server "${cpp_typesupport_target}")

install(TARGETS arm_motion_plan_server arm_joint_motion_plan_server
  DESTINATION lib/${PROJECT_NAME}
)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "action/GoToPose.action"
#  "action/GoToJoint.action"
#  "action/GripperControl.action"
#  "action/RobotiqGripperControl.action"
#  "action/CircMotion.action"
#  DEPENDENCIES geometry_msgs
#)

#add_library(go_to_pose_action_server SHARED src/go_to_pose_action_server.cpp)
#target_include_directories(go_to_pose_action_server PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
#)
#ament_target_dependencies(go_to_pose_action_server
#  rclcpp
#  rclcpp_action
#  rclcpp_components
#  moveit_ros_planning_interface
#  moveit_ros_warehouse
#  moveit_msgs
#  geometry_msgs
#  sensor_msgs
#  trajectory_msgs
#  warehouse_ros
#)


install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
#install(PROGRAMS
#  scripts/robotiq_gripper.py
#  scripts/robotiq_gripper_action_server.py
#  scripts/robotiq_gripper_action_client.py
#  scripts/ur_external_control_monitor.py
#  DESTINATION lib/${PROJECT_NAME}
#)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
