cmake_minimum_required(VERSION 3.16)
project(usb_driver)

## Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

include_directories(include)

target_include_directories(usb_driver PUBLIC
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(usb_driver
  ${LIBUSB_LINK_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN usb_driver::UsbDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)

