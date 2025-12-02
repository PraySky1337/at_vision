cmake_minimum_required(VERSION 3.10)
project(armor_detector_ov)

## Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-O3)


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXTERNAL_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${G2O_LIBRARIES})

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(G2O REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
g2o_core
g2o_stuff
g2o_solver_csparse
g2o_types_sba
g2o_types_slam3d
g2o_solver_dense
openvino::runtime
)


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
