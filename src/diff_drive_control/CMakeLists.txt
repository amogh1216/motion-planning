cmake_minimum_required(VERSION 3.8)
project(diff_drive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)

install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(diff_drive_brain src/diff_drive_brain.cpp)
add_executable(localization_node src/localization.cpp)

target_include_directories(diff_drive_brain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(diff_drive_brain PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(localization_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(diff_drive_brain  
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_msgs
)

ament_target_dependencies(localization_node  
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_msgs
)

install(TARGETS diff_drive_brain localization_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/path_following/pure_pursuit.py
  DESTINATION lib/${PROJECT_NAME}
)

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
