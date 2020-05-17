cmake_minimum_required(VERSION 3.5)
project(robotx_twist_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${CERESS_INCLUDE_DIRS}
)

add_library(robotx_twist_controller_component SHARED
  src/robotx_twist_controller_component.cpp
)
target_compile_definitions(robotx_twist_controller_component PRIVATE "ROBOTX_TWIST_CONTROLLER_ROBOTX_TWIST_CONTROLLER_COMPONENT_BUILDING_DLL")
target_link_libraries(robotx_twist_controller_component ${CERES_LIBRARIES})
ament_target_dependencies(robotx_twist_controller_component
  rclcpp rclcpp_components geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(robotx_twist_controller_node
  src/robotx_twist_controller_node.cpp
)
target_link_libraries(robotx_twist_controller_node robotx_twist_controller_component ${CERES_LIBRARIES})
ament_target_dependencies(robotx_twist_controller_node
  rclcpp rclcpp_components geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

rclcpp_components_register_nodes(robotx_twist_controller_component
  "robotx_twist_controller::RobotXTwistControllerComponent")

install(TARGETS
  robotx_twist_controller_node
  DESTINATION lib/robotx_twist_controller
)
install(TARGETS
  robotx_twist_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(robotx_twist_controller_component)

#############
## Install ##
#############
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
