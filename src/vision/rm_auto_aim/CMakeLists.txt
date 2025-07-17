cmake_minimum_required(VERSION 3.8)
project(rm_auto_aim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(
    INSTALL_TO_SHARE
    config
    launch
)
