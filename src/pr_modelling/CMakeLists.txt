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
find_package(message_filters REQUIRED)

find_package(pr_msgs REQUIRED)
find_package(pr_lib REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

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


### Independent Jacobian node ###

add_library(ind_jac SHARED
  src/ind_jac.cpp)

target_compile_definitions(ind_jac
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ind_jac
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(ind_jac "pr_modelling::IndependentJacobian")
set(node_plugins "${node_plugins}pr_modelling::IndependentJacobian;$<TARGET_FILE:ind_jac>\n")



### Dependent Jacobian node ###

add_library(dep_jac SHARED
  src/dep_jac.cpp)

target_compile_definitions(dep_jac
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dep_jac
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(dep_jac "pr_modelling::DependentJacobian")
set(node_plugins "${node_plugins}pr_modelling::DependentJacobian;$<TARGET_FILE:dep_jac>\n")


### RastT node ###

add_library(rast_t SHARED
  src/rast_t.cpp)

target_compile_definitions(rast_t
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(rast_t
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(rast_t "pr_modelling::RastT")
set(node_plugins "${node_plugins}pr_modelling::RastT;$<TARGET_FILE:rast_t>\n")


### QGrav terms node ###

add_library(q_grav SHARED
  src/q_grav.cpp)

target_compile_definitions(q_grav
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(q_grav
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(q_grav "pr_modelling::QGrav")
set(node_plugins "${node_plugins}pr_modelling::QGrav;$<TARGET_FILE:q_grav>\n")


### STATE PUBLISHER node ###

add_library(state_publisher SHARED
  src/state_publisher.cpp)

target_compile_definitions(state_publisher
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(state_publisher
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs")

rclcpp_components_register_nodes(state_publisher "pr_modelling::StatePublisher")
set(node_plugins "${node_plugins}pr_modelling::StatePublisher;$<TARGET_FILE:state_publisher>\n")


### ANG OTS PUBLISHER node ###

add_library(ang_ots SHARED
  src/ang_ots.cpp)

target_compile_definitions(ang_ots
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ang_ots
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib")

rclcpp_components_register_nodes(ang_ots "pr_modelling::AngOTS")
set(node_plugins "${node_plugins}pr_modelling::StatePublisher;$<TARGET_FILE:ang_ots>\n")


### Installation ###

install(TARGETS
  for_kin
  inv_kin
  ind_jac
  dep_jac
  rast_t
  q_grav
  state_publisher
  ang_ots
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()