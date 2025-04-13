cmake_minimum_required(VERSION 3.8)
project(niryo_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(object_recognition_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/NiryoMoveitJoints.msg"
    "msg/RobotState.msg"
    "msg/AttachedCollisionObject.msg"
    "msg/CollisionObject.msg"
    "msg/RobotTrajectory.msg"
    "msg/NiryoTrajectory.msg"
    "srv/MoverService.srv"
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs trajectory_msgs object_recognition_msgs shape_msgs 
 )
ament_export_dependencies(rosidl_default_runtime)

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

install(
  DIRECTORY
    scripts
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/trajectory_subscriber.py
  launch/trajectory_subscriber.launch.py
  scripts/mover.py
  launch/mover.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
