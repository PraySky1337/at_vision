cmake_minimum_required(VERSION 3.10)
project(rune_solver)

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
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
ament_auto_find_build_dependencies()


###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)


target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::rune::RuneSolverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)

