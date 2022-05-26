cmake_minimum_required(VERSION 3.8)
project(ros2_camera_aravis)
include(FindPkgConfig)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
message(TRACE "testing")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)

pkg_search_module(ARAVIS REQUIRED aravis-0.8 aravis-0.6 aravis-0.4 aravis-0.2)
pkg_check_modules(GLIB REQUIRED glib-2.0)

#${CMAKE_CURRENT_SOURCE_DIR}/

file(GLOB msg_files
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
  "msg/*.msg"
)
file(GLOB srv_files
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/"
  "srv/*.srv"
)

message(TRACE "Message files found: ${msg_files}")
message(TRACE "Service files found: ${srv_files}")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  ${ARAVIS_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
)

add_library("${PROJECT_NAME}-lib"
  src/camera_aravis_nodelet.cpp
  src/camera_buffer_pool.cpp
)

target_include_directories("${PROJECT_NAME}-lib" PRIVATE ${GLIB_INCLUDE_DIRS})

ament_package()
