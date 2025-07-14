cmake_minimum_required(VERSION 3.8)
project(rm_auto_aim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

find_package(PkgConfig)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
ament_auto_find_build_dependencies()


ament_auto_add_executable(rm_auto_aim
    DIRECTORY src)

include_directories(SYSTEM "/usr/include/libusb-1.0")

target_link_libraries(${PROJECT_NAME} -lusb-1.0)

ament_auto_package(
    INSTALL_TO_SHARE
    config
    launch
)
