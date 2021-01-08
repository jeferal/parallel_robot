cmake_minimum_required(VERSION 3.5)
project(pr_main)

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

find_package(pr_sensors_actuators REQUIRED)
find_package(pr_ref_gen REQUIRED)
find_package(pr_controllers REQUIRED)
find_package(pr_modelling REQUIRED)


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

add_executable(pr_gus_exe "src/pr_gus_main.cpp")

ament_target_dependencies(pr_gus_exe
  rclcpp
  encoders_sim
  ref_pose
  derivator
  gus_controller
)

install(TARGETS pr_gus_exe
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/pr_main
  INCLUDES DESTINATION include
)


ament_package()
