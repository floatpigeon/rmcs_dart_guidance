cmake_minimum_required(VERSION 3.12)
project(rmcs_dart_guidance)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)


find_package(OpenCV 4.5 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

find_package(type_description_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

find_package(rmcs_executor REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(fast_tf REQUIRED)
find_package(hikcamera REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/src/

  ${type_description_interfaces_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmcs_executor_INCLUDE_DIRS}
  ${rmcs_description_INCLUDE_DIRS}
  ${fast_tf_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

file(GLOB_RECURSE PROJECT_SOURCE
  CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.cc
  ${PROJECT_SOURCE_DIR}/test/*.cpp
  ${PROJECT_SOURCE_DIR}/test/*.cc
)

add_library(
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rosidl_typesupport_cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${rmcs_executor_LIBRARIES}
)

install(
  TARGETS 
    ${PROJECT_NAME}
  DESTINATION 
    lib/
)

find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(
  rmcs_executor plugins.xml
)

find_package(ament_cmake REQUIRED)
ament_package()