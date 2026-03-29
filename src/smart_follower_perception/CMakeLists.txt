cmake_minimum_required(VERSION 3.8)
project(smart_follower_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(smart_follower_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(astra_camera_msgs REQUIRED)

set(SMART_FOLLOWER_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party")
set(SMART_FOLLOWER_WHEELTEC_THIRD_PARTY_DIR "$ENV{HOME}/wheeltec_ros2/third_party")
set(SMART_FOLLOWER_WHEELTEC_ABS_THIRD_PARTY_DIR "/home/wheeltec/wheeltec_ros2/third_party")

set(ONNXRUNTIME_HINTS
  $ENV{ONNXRUNTIME_ROOT}
  ${SMART_FOLLOWER_WHEELTEC_ABS_THIRD_PARTY_DIR}/onnxruntime-linux-aarch64-1.24.3
  ${SMART_FOLLOWER_WHEELTEC_ABS_THIRD_PARTY_DIR}/onnxruntime-linux-aarch64
  ${SMART_FOLLOWER_WHEELTEC_ABS_THIRD_PARTY_DIR}/onnxruntime
  ${SMART_FOLLOWER_WHEELTEC_THIRD_PARTY_DIR}/onnxruntime-linux-aarch64-1.24.3
  ${SMART_FOLLOWER_WHEELTEC_THIRD_PARTY_DIR}/onnxruntime-linux-aarch64
  ${SMART_FOLLOWER_WHEELTEC_THIRD_PARTY_DIR}/onnxruntime
  ${SMART_FOLLOWER_THIRD_PARTY_DIR}/onnxruntime-linux-aarch64-1.24.3
  ${SMART_FOLLOWER_THIRD_PARTY_DIR}/onnxruntime-linux-aarch64
  ${SMART_FOLLOWER_THIRD_PARTY_DIR}/onnxruntime
  $ENV{HOME}/opt/onnxruntime-linux-aarch64-1.24.3
  $ENV{HOME}/opt/onnxruntime-linux-x64-1.23.2
  /usr
  /usr/local
)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_HINTS}
  PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session
)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS ${ONNXRUNTIME_HINTS}
  PATH_SUFFIXES lib lib64
)

add_library(smart_follower_perception_core
  src/runtime.cpp
  src/frame_sync.cpp
  src/tracker.cpp
  src/lock_manager.cpp
  src/geometry_utils.cpp
  src/pipeline_utils.cpp
  src/perception_params.cpp
  src/perception_diagnostics.cpp
  src/perception_pipeline.cpp
)

target_include_directories(smart_follower_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(smart_follower_perception_core
  ${OpenCV_LIBS}
)

ament_target_dependencies(smart_follower_perception_core
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  std_msgs
  diagnostic_updater
  diagnostic_msgs
  smart_follower_msgs
  cv_bridge
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  target_compile_definitions(smart_follower_perception_core PUBLIC HAVE_ONNXRUNTIME=1)
  target_include_directories(smart_follower_perception_core PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(smart_follower_perception_core ${ONNXRUNTIME_LIBRARY})
  get_filename_component(ONNXRUNTIME_LIBRARY_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
  set_target_properties(smart_follower_perception_core PROPERTIES
    BUILD_RPATH ${ONNXRUNTIME_LIBRARY_DIR}
    INSTALL_RPATH ${ONNXRUNTIME_LIBRARY_DIR}
  )
else()
  message(WARNING "ONNX Runtime not found. perception_node will build with stub inference.")
endif()

add_executable(perception_node
  src/perception_node.cpp
)

target_include_directories(perception_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(perception_node
  smart_follower_perception_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

ament_target_dependencies(perception_node astra_camera_msgs)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  set_target_properties(perception_node PROPERTIES
    BUILD_RPATH ${ONNXRUNTIME_LIBRARY_DIR}
    INSTALL_RPATH ${ONNXRUNTIME_LIBRARY_DIR}
  )
endif()

ament_target_dependencies(perception_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  diagnostic_updater
  diagnostic_msgs
  smart_follower_msgs
)

install(TARGETS perception_node smart_follower_perception_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()

  ament_add_gtest(test_assignment test/test_assignment.cpp)
  if(TARGET test_assignment)
    target_include_directories(test_assignment PRIVATE include)
    target_link_libraries(test_assignment Eigen3::Eigen)
  endif()

  ament_add_gtest(test_frame_sync test/test_frame_sync.cpp)
  if(TARGET test_frame_sync)
    target_include_directories(test_frame_sync PRIVATE include)
    ament_target_dependencies(test_frame_sync rclcpp sensor_msgs)
    target_link_libraries(test_frame_sync smart_follower_perception_core)
  endif()

  ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp)
  if(TARGET test_geometry_utils)
    target_include_directories(test_geometry_utils PRIVATE include)
    ament_target_dependencies(test_geometry_utils geometry_msgs)
    target_link_libraries(test_geometry_utils smart_follower_perception_core)
  endif()

  ament_add_gtest(test_lock_manager test/test_lock_manager.cpp)
  if(TARGET test_lock_manager)
    target_include_directories(test_lock_manager PRIVATE include)
    ament_target_dependencies(test_lock_manager rclcpp smart_follower_msgs)
    target_link_libraries(test_lock_manager smart_follower_perception_core)
  endif()

  ament_add_gtest(test_tracker test/test_tracker.cpp)
  if(TARGET test_tracker)
    target_include_directories(test_tracker PRIVATE include)
    ament_target_dependencies(test_tracker rclcpp smart_follower_msgs)
    target_link_libraries(test_tracker smart_follower_perception_core)
  endif()
endif()

ament_package()
