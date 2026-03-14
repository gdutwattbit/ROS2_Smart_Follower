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
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(smart_follower_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(SMART_FOLLOWER_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party")

set(ONNXRUNTIME_HINTS
  $ENV{ONNXRUNTIME_ROOT}
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

add_executable(perception_node
  src/perception_node.cpp
)

target_include_directories(perception_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  target_compile_definitions(perception_node PRIVATE HAVE_ONNXRUNTIME=1)
  target_include_directories(perception_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(perception_node ${ONNXRUNTIME_LIBRARY})
  get_filename_component(ONNXRUNTIME_LIBRARY_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
  set_target_properties(perception_node PROPERTIES
    BUILD_RPATH ${ONNXRUNTIME_LIBRARY_DIR}
    INSTALL_RPATH ${ONNXRUNTIME_LIBRARY_DIR}
  )
else()
  message(WARNING "ONNX Runtime not found. perception_node will build with stub inference.")
endif()

target_link_libraries(perception_node ${OpenCV_LIBS} Eigen3::Eigen)

ament_target_dependencies(perception_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_updater
  diagnostic_msgs
  smart_follower_msgs
)

install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_assignment test/test_assignment.cpp)
  if(TARGET test_assignment)
    target_include_directories(test_assignment PRIVATE include)
    target_link_libraries(test_assignment Eigen3::Eigen)
  endif()
endif()

ament_package()


