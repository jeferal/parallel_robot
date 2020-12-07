cmake_minimum_required(VERSION 3.5)
project(pr_modelling)

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
find_package(rcutils REQUIRED)

find_package(pr_msgs REQUIRED)
find_package(pr_lib REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(/home/paralelo4dofnew/parallel_robot_ws/src/pr_lib/include)

set(node_plugins "")

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

### Forward Kinematics node ###

add_library(for_kin SHARED
  src/for_kin.cpp)

target_compile_definitions(for_kin
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(for_kin
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(for_kin "pr_modelling::ForwardKinematics")
set(node_plugins "${node_plugins}pr_modelling::ForwardKinematics;$<TARGET_FILE:for_kin>\n")


### Inverse Kinematics node ###

add_library(inv_kin SHARED
  src/inv_kin.cpp)

target_compile_definitions(inv_kin
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(inv_kin
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(inv_kin "pr_modelling::InverseKinematics")
set(node_plugins "${node_plugins}pr_modelling::InverseKinematics;$<TARGET_FILE:inv_kin>\n")


### Installation ###

install(TARGETS
  for_kin
  inv_kin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()