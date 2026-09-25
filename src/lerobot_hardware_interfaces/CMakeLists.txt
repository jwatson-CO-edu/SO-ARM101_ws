cmake_minimum_required(VERSION 3.8)
project(lerobot_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces (not a plain ament_python install) is required here because
# custom .action/.msg/.srv files need code generation; that generation step only runs from
# an ament_cmake package, which is why this action lives in its own package rather than
# alongside the Python nodes in lerobot_hardware.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
