cmake_minimum_required(VERSION 3.10)

project(libusb_test)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})
add_definitions(${LIBUSB_CFLAGS_OTHER})
link_libraries(${LIBUSB_LIBRARIES})

add_executable(libusb_test usb_test.cpp)

