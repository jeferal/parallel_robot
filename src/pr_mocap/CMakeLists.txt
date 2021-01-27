cmake_minimum_required(VERSION 3.5)
project(pr_mocap)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pr_lib REQUIRED)

find_library(/home/paralelo4dofnew/ros2_eloquent_ws/parallel_robot/src/pr_mocap/natnet_sdk/lib NatNetLibShared)

include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/parallel_robot/src/pr_mocap/natnet_sdk/include)

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

### Mocap node ###

add_library(pr_x_mocap SHARED
  src/pr_x_mocap.cpp)

target_link_libraries(pr_x_mocap "-L/home/paralelo4dofnew/ros2_eloquent_ws/parallel_robot/src/pr_mocap/natnet_sdk/lib" NatNetLibShared)

target_compile_definitions(pr_x_mocap
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pr_x_mocap
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3"
)

rclcpp_components_register_nodes(pr_x_mocap "pr_mocap::PRXMocap")
set(node_plugins "${node_plugins}pr_mocap::PRXMocap;$<TARGET_FILE:pr_x_mocap>\n")

### Mocap error node ###

add_library(error_model SHARED
  src/error_model.cpp)

target_compile_definitions(error_model
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(error_model
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
)

rclcpp_components_register_nodes(error_model "pr_mocap::ErrorModel")
set(node_plugins "${node_plugins}pr_mocap::ErrorModel;$<TARGET_FILE:error_model>\n")

install(TARGETS
  pr_x_mocap
  error_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

ament_package()
