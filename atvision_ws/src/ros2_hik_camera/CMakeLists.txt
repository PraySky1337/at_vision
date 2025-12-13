cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC hikSDK/include)

# Allow overriding target architecture so the package can be built for multiple platforms
set(HIK_SDK_ARCH "" CACHE STRING "Target architecture for Hik SDK (amd64 or arm64)")
if(NOT HIK_SDK_ARCH AND DEFINED ENV{HIK_SDK_ARCH})
  set(HIK_SDK_ARCH "$ENV{HIK_SDK_ARCH}")
endif()

if(NOT HIK_SDK_ARCH)
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _hik_cpu)
  if(_hik_cpu MATCHES "x86_64|amd64")
    set(HIK_SDK_ARCH "amd64")
  elseif(_hik_cpu MATCHES "aarch64|arm64")
    set(HIK_SDK_ARCH "arm64")
  else()
    message(
      FATAL_ERROR
        "Unsupported target architecture '${CMAKE_SYSTEM_PROCESSOR}'. "
        "Set HIK_SDK_ARCH to one of: amd64, arm64.")
  endif()
endif()

set(_hik_sdk_lib_dir "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/${HIK_SDK_ARCH}")
if(NOT EXISTS "${_hik_sdk_lib_dir}")
  message(FATAL_ERROR "Hik SDK libraries not found for '${HIK_SDK_ARCH}' at ${_hik_sdk_lib_dir}")
endif()

target_link_directories(${PROJECT_NAME} PUBLIC ${_hik_sdk_lib_dir})
install(
  DIRECTORY ${_hik_sdk_lib_dir}/
  DESTINATION lib
)

target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

set(USE_SCOPED_HEADER_INSTALL_DIR ON)
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
