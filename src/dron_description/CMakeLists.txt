cmake_minimum_required(VERSION 3.8)
project(dron_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
