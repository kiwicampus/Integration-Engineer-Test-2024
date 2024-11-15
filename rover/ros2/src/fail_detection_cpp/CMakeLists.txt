cmake_minimum_required(VERSION 3.8)
project(fail_detection_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(usr_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)
include_directories(SYSTEM)

set(DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  usr_msgs
  utils
  tf2
)

add_library(fail_detection 
  src/fail_detection.cpp
)

add_executable(fail_detection_node
  src/fail_detection_node.cpp
)

target_link_libraries(fail_detection utils::utils)
ament_target_dependencies(fail_detection ${DEPENDENCIES})

target_link_libraries(fail_detection_node
  utils::utils
  fail_detection
)

ament_target_dependencies(fail_detection_node
  ${DEPENDENCIES}
)

install(TARGETS fail_detection_node
  DESTINATION lib/${PROJECT_NAME})

  install(
  TARGETS fail_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()
