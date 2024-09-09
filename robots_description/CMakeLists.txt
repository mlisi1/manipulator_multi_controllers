cmake_minimum_required(VERSION 3.5)
project(robots_description)

find_package(ament_cmake)


install(
  DIRECTORY robots launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
