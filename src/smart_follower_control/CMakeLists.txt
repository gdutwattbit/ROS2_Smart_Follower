cmake_minimum_required(VERSION 3.8)
project(smart_follower_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(smart_follower_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(SMART_FOLLOWER_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party")
set(PIGPIO_HINTS
  $ENV{PIGPIO_ROOT}
  ${SMART_FOLLOWER_THIRD_PARTY_DIR}/pigpio
  /usr
  /usr/local
)
find_path(PIGPIO_INCLUDE_DIR pigpio.h
  HINTS ${PIGPIO_HINTS}
  PATH_SUFFIXES include include/pigpio
)
find_library(PIGPIO_LIBRARY pigpio
  HINTS ${PIGPIO_HINTS}
  PATH_SUFFIXES lib lib64 lib/aarch64-linux-gnu
)

set(common_deps
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  std_msgs
  diagnostic_updater
  diagnostic_msgs
  smart_follower_msgs
)

add_executable(follower_controller_node src/follower_controller_node.cpp)
ament_target_dependencies(follower_controller_node ${common_deps})
target_include_directories(follower_controller_node PRIVATE include)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node ${common_deps})
target_include_directories(obstacle_avoidance_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(obstacle_avoidance_node ${OpenCV_LIBS})

add_executable(arbiter_node src/arbiter_node.cpp)
ament_target_dependencies(arbiter_node ${common_deps})
target_include_directories(arbiter_node PRIVATE include)

add_executable(keyboard_command_node src/keyboard_command_node.cpp)
ament_target_dependencies(keyboard_command_node rclcpp smart_follower_msgs)
target_include_directories(keyboard_command_node PRIVATE include)

add_executable(ultrasonic_range_node src/ultrasonic_range_node.cpp)
ament_target_dependencies(ultrasonic_range_node ${common_deps})
target_include_directories(ultrasonic_range_node PRIVATE include)
if(PIGPIO_INCLUDE_DIR AND PIGPIO_LIBRARY)
  target_compile_definitions(ultrasonic_range_node PRIVATE HAVE_PIGPIO=1)
  target_include_directories(ultrasonic_range_node PRIVATE ${PIGPIO_INCLUDE_DIR})
  target_link_libraries(ultrasonic_range_node ${PIGPIO_LIBRARY})
  get_filename_component(PIGPIO_LIBRARY_DIR ${PIGPIO_LIBRARY} DIRECTORY)
  set_target_properties(ultrasonic_range_node PROPERTIES
    BUILD_RPATH ${PIGPIO_LIBRARY_DIR}
    INSTALL_RPATH ${PIGPIO_LIBRARY_DIR}
  )
else()
  message(WARNING "pigpio not found. ultrasonic_range_node will run in dry mode.")
endif()

install(TARGETS
  follower_controller_node
  obstacle_avoidance_node
  arbiter_node
  keyboard_command_node
  ultrasonic_range_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arbiter_sm test/test_arbiter_sm.cpp)
  if(TARGET test_arbiter_sm)
    target_include_directories(test_arbiter_sm PRIVATE include)
  endif()
endif()

ament_package()



