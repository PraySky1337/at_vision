cmake_minimum_required(VERSION 3.8)
project(solver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_definitions(-Wall -Werror)
add_subdirectory(TinyMPC)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN solver::Trajectory
  EXECUTABLE trajectory_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN solver::Planner
  EXECUTABLE planner_node
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen tinympcstatic)
target_compile_options(solver PRIVATE -Wno-return-type-c-linkage)
# 忽略 extern "C" {} 的警告

ament_auto_package()
