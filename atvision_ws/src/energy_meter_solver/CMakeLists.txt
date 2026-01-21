cmake_minimum_required(VERSION 3.10)
project(energy_meter_solver)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()


###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/angle_fitter.cpp
  src/energy_meter_solver_node.cpp
  src/energy_meter_tracker.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN energy_meter::EnergyMeterSolverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

set(USE_SCOPED_HEADER_INSTALL_DIR ON)
ament_auto_package(
  INSTALL_TO_SHARE
)
