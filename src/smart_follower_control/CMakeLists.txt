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
set(LIBGPIOD_HINTS
  $ENV{LIBGPIOD_ROOT}
  ${SMART_FOLLOWER_THIRD_PARTY_DIR}/libgpiod
  ${SMART_FOLLOWER_THIRD_PARTY_DIR}/libgpiod/install
  /usr
  /usr/local
)
find_path(LIBGPIOD_INCLUDE_DIR gpiod.h
  HINTS ${LIBGPIOD_HINTS}
  PATH_SUFFIXES include include/gpiod
)
find_library(LIBGPIOD_LIBRARY gpiod
  HINTS ${LIBGPIOD_HINTS}
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

add_library(smart_follower_control_core
  src/arbiter_runtime.cpp
  src/follower_runtime.cpp
  src/obstacle_runtime.cpp
  src/ultrasonic_runtime.cpp
)
ament_target_dependencies(smart_follower_control_core
  rclcpp
  geometry_msgs
  sensor_msgs
  smart_follower_msgs
  cv_bridge
)
target_include_directories(smart_follower_control_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(smart_follower_control_core ${OpenCV_LIBS})

add_executable(follower_controller_node src/follower_controller_node.cpp)
ament_target_dependencies(follower_controller_node ${common_deps})
target_include_directories(follower_controller_node PRIVATE include)
target_link_libraries(follower_controller_node smart_follower_control_core)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node ${common_deps})
target_include_directories(obstacle_avoidance_node PRIVATE include)
target_link_libraries(obstacle_avoidance_node smart_follower_control_core)

add_executable(arbiter_node src/arbiter_node.cpp)
ament_target_dependencies(arbiter_node ${common_deps})
target_include_directories(arbiter_node PRIVATE include)
target_link_libraries(arbiter_node smart_follower_control_core)

add_executable(keyboard_command_node src/keyboard_command_node.cpp)
ament_target_dependencies(keyboard_command_node rclcpp smart_follower_msgs)
target_include_directories(keyboard_command_node PRIVATE include)

add_executable(ultrasonic_range_node src/ultrasonic_range_node.cpp)
ament_target_dependencies(ultrasonic_range_node ${common_deps})
target_include_directories(ultrasonic_range_node PRIVATE include)
target_link_libraries(ultrasonic_range_node smart_follower_control_core)
if(LIBGPIOD_INCLUDE_DIR AND LIBGPIOD_LIBRARY)
  target_compile_definitions(smart_follower_control_core PRIVATE HAVE_LIBGPIOD=1)
  target_include_directories(smart_follower_control_core PRIVATE ${LIBGPIOD_INCLUDE_DIR})
  target_link_libraries(smart_follower_control_core ${LIBGPIOD_LIBRARY})
  get_filename_component(LIBGPIOD_LIBRARY_DIR ${LIBGPIOD_LIBRARY} DIRECTORY)
  set_target_properties(ultrasonic_range_node PROPERTIES
    BUILD_RPATH ${LIBGPIOD_LIBRARY_DIR}
    INSTALL_RPATH ${LIBGPIOD_LIBRARY_DIR}
  )
else()
  message(WARNING "libgpiod not found. ultrasonic_range_node will run in dry mode.")
endif()

install(TARGETS
  smart_follower_control_core
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
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()


  ament_add_gtest(test_arbiter_sm test/test_arbiter_sm.cpp)
  if(TARGET test_arbiter_sm)
    target_include_directories(test_arbiter_sm PRIVATE include)
  endif()

  ament_add_gtest(test_arbiter_runtime test/test_arbiter_runtime.cpp)
  if(TARGET test_arbiter_runtime)
    target_include_directories(test_arbiter_runtime PRIVATE include)
    ament_target_dependencies(test_arbiter_runtime rclcpp geometry_msgs smart_follower_msgs)
    target_link_libraries(test_arbiter_runtime smart_follower_control_core)
  endif()

  ament_add_gtest(test_follower_runtime test/test_follower_runtime.cpp)
  if(TARGET test_follower_runtime)
    target_include_directories(test_follower_runtime PRIVATE include)
    ament_target_dependencies(test_follower_runtime rclcpp geometry_msgs smart_follower_msgs)
    target_link_libraries(test_follower_runtime smart_follower_control_core)
  endif()

  ament_add_gtest(test_obstacle_runtime test/test_obstacle_runtime.cpp)
  if(TARGET test_obstacle_runtime)
    target_include_directories(test_obstacle_runtime PRIVATE include)
    ament_target_dependencies(test_obstacle_runtime rclcpp geometry_msgs sensor_msgs cv_bridge)
    target_link_libraries(test_obstacle_runtime smart_follower_control_core)
  endif()
endif()

ament_package()
