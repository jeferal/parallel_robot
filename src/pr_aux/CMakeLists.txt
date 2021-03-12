cmake_minimum_required(VERSION 3.5)
project(pr_aux)

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

### Derivator node ###

add_library(derivator SHARED
  src/derivator.cpp)

target_compile_definitions(derivator
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(derivator
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(derivator "pr_aux::Derivator")
set(node_plugins "${node_plugins}pr_aux::Derivator;$<TARGET_FILE:derivator>\n")

### Replayer node ###

add_library(replayer SHARED
  src/replayer.cpp)

target_compile_definitions(replayer
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(replayer
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib")

rclcpp_components_register_nodes(replayer "pr_aux::Replayer")
set(node_plugins "${node_plugins}pr_aux::Replayer;$<TARGET_FILE:replayer>\n")


### Installation ###

install(TARGETS
  derivator
  replayer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()
