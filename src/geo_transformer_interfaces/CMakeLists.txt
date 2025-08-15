cmake_minimum_required(VERSION 3.8)
project(geo_transformer_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter Development)
find_package(builtin_interfaces REQUIRED)   # <-- add this

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetOrigin.srv"
  "srv/GetOrigin.srv"
  "srv/FromLL.srv"
  "srv/ToLL.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
