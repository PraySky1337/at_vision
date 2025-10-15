cmake_minimum_required(VERSION 3.16)
project(rm_gimbal)

## Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_subdirectory(tinympc)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

include_directories(include)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Werror
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${LIBUSB_LINK_LIBRARIES}
  tinympcstatic
)

install(DIRECTORY
        urdf
        DESTINATION share/${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_gimbal::GimbalNode
  EXECUTABLE ${PROJECT_NAME}_node
)


set(USE_SCOPED_HEADER_INSTALL_DIR ON)
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)

