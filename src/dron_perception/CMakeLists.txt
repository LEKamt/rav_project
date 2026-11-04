cmake_minimum_required(VERSION 3.8)
project(dron_perception)

find_package(ament_cmake REQUIRED)

# Paquete reservado: todavia no instala nada. Ver README.md.
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
