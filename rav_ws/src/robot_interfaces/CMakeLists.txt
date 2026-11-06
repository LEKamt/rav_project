cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EstadoDron.msg"
  "msg/EstadoAltura.msg"
  "msg/SenalesControl.msg"
  "srv/Armar.srv"
  "action/IrAAltura.action"
  DEPENDENCIES std_msgs
)

ament_package()
