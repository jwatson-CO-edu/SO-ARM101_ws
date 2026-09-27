cmake_minimum_required(VERSION 3.8)
project(lerobot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lerobot_perception_interfaces REQUIRED)

# OpenCV and PCL are plain CMake (non-ament) packages: they export <name>_INCLUDE_DIRS /
# <name>_LIBRARIES (PCL) or _LIBS (OpenCV) variables, not the ament-index metadata
# ament_target_dependencies() expects, so they're linked directly below instead.
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree)
# Parallelizes buildCloud()'s depth->cloud pixel loop (see color_point_cloud_service_node.cpp).
find_package(OpenMP REQUIRED)

add_executable(color_point_cloud_service_node
  src/color_point_cloud_service_node.cpp
  src/color_point_cloud_service_node_main.cpp
)

target_include_directories(color_point_cloud_service_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(color_point_cloud_service_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(color_point_cloud_service_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(color_point_cloud_service_node
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  pcl_conversions
  lerobot_perception_interfaces
)

install(TARGETS
  color_point_cloud_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
