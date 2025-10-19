cmake_minimum_required(VERSION 3.10)
project(cafe_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MenuItem.msg"
  "msg/RobotStatus.msg"
  "srv/GetMenu.srv"
  "srv/CheckPrice.srv"
  "srv/CancelOrder.srv"
  "action/MakeDrink.action"
  "action/DeliverOrder.action"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(python_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_python")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
