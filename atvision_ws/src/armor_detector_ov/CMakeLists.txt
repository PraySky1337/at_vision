cmake_minimum_required(VERSION 3.10)
project(armor_detector_ov)

## Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compiler flags
add_definitions(-O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Build Options     ##
#######################

option(ENABLE_OPENVINO "Enable OpenVINO NN backend" ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
ament_auto_find_build_dependencies()

# Conditionally find OpenVINO
if(ENABLE_OPENVINO)
  find_package(OpenVINO QUIET)
  if(NOT OpenVINO_FOUND)
    message(WARNING "OpenVINO not found. NN backend will be disabled.")
    set(ENABLE_OPENVINO OFF)
  else()
    find_package(TBB QUIET COMPONENTS tbb)
  endif()
endif()

###########
## Build ##
###########

# Base sources (always compiled)
set(BASE_SOURCES
  src/detector/detector_node.cpp
  src/detector/pnp_solver.cpp
  src/detector/traditional_backend.cpp
  src/detector/number_classifier.cpp
)

# OpenVINO sources (only when enabled)
if(ENABLE_OPENVINO)
  set(OPENVINO_SOURCES
    src/detector/nn_backend.cpp
    src/detector/inference/ov_armor_tup.cpp
    src/detector/inference/ov_armor_at.cpp
  )
  message(STATUS "Building with OpenVINO NN backend support")
else()
  set(OPENVINO_SOURCES "")
  message(STATUS "Building WITHOUT OpenVINO (traditional backend only)")
endif()

# Build the library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${BASE_SOURCES}
  ${OPENVINO_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

# Conditionally link OpenVINO
if(ENABLE_OPENVINO)
  target_link_libraries(${PROJECT_NAME}
    openvino::runtime
  )
  if(TBB_FOUND)
    target_link_libraries(${PROJECT_NAME} TBB::tbb)
  endif()
  target_compile_definitions(${PROJECT_NAME} PUBLIC ENABLE_OPENVINO)
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorOVNode
  EXECUTABLE armor_detector_ov_node
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
  model
)

# Build summary
message(STATUS "=== armor_detector_ov Build Configuration ===")
message(STATUS "  OpenVINO Backend: ${ENABLE_OPENVINO}")
message(STATUS "  OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "=============================================")
