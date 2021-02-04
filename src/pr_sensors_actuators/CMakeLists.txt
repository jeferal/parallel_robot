cmake_minimum_required(VERSION 3.5)
project(pr_sensors_actuators)

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

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(pr_msgs REQUIRED)

find_library(BIODAQ_LIBRARY NAMES biodaq)

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

### Encoders node ###

add_library(encoders SHARED
  src/encoders.cpp)

target_compile_definitions(encoders
  PRIVATE "COMPOSITION_BUILDING_DLL")

target_link_libraries(encoders biodaq)

ament_target_dependencies(encoders
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "pr_msgs")

rclcpp_components_register_nodes(encoders "pr_sensors_actuators::Encoders")
set(node_plugins "${node_plugins}pr_sensors_actuators::Encoders;$<TARGET_FILE:encoders>\n")


### Motor node ###

add_library(motor SHARED
  src/motor.cpp)

target_compile_definitions(motor
  PRIVATE "COMPOSITION_BUILDING_DLL")

target_link_libraries(motor biodaq)

ament_target_dependencies(motor
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "pr_msgs")

rclcpp_components_register_nodes(motor "pr_sensors_actuators::Motor")
set(node_plugins "${node_plugins}pr_sensors_actuators::Motor;$<TARGET_FILE:motor>\n")


### Encoders sim node ###

add_library(encoders_sim SHARED
  src/encoders_sim.cpp)

target_compile_definitions(encoders_sim
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(encoders_sim
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "pr_msgs")

rclcpp_components_register_nodes(encoders_sim "pr_sensors_actuators::EncodersSim")
set(node_plugins "${node_plugins}pr_sensors_actuators::EncodersSim;$<TARGET_FILE:encoders_sim>\n")


### Force sensor node ###

add_library(force_sensor SHARED
  src/force_sensor.cpp)

target_compile_definitions(force_sensor
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(force_sensor
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(force_sensor "pr_sensors_actuators::ForceSensor")
set(node_plugins "${node_plugins}pr_sensors_actuators::ForceSensor;$<TARGET_FILE:force_sensor>\n")



### Encoders Simulink node ###

add_library(encoders_simulink SHARED
  src/simulated/encoders_simulink.cpp)

target_compile_definitions(encoders_simulink
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(encoders_simulink
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "pr_msgs"
  "geometry_msgs")

rclcpp_components_register_nodes(encoders_simulink "pr_sensors_actuators::EncodersSimulink")
set(node_plugins "${node_plugins}pr_sensors_actuators::EncodersSimulink;$<TARGET_FILE:encoders_simulink>\n")



### Installation ###

install(TARGETS
  encoders
  motor
  encoders_sim
  force_sensor
  encoders_simulink
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


### Install launch files ###
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
