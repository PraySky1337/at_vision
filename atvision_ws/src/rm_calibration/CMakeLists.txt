cmake_minimum_required(VERSION 3.8)
project(rm_calibration)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(camera_info_manager REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}_lib
  src/intrinsic_calibrator.cpp
  src/extrinsic_calibrator.cpp
  src/quality_scorer.cpp
)

set_target_properties(${PROJECT_NAME}_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  camera_info_manager
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

# Create composable node library
add_library(${PROJECT_NAME}_component SHARED
  src/calibration_node.cpp
)

target_compile_definitions(${PROJECT_NAME}_component PRIVATE RM_CALIBRATION_COMPONENT)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  camera_info_manager
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rm_calibration::CalibrationNode"
  EXECUTABLE calibration_node
  EXECUTOR MultiThreadedExecutor
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
