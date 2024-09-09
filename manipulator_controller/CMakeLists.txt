cmake_minimum_required(VERSION 3.16)
project(manipulator_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
  Eigen3
  std_msgs
  sensor_msgs
  geometry_msgs
  kdl_parser
  urdf
  controller_error_msgs
)


find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()    




add_library(
    ${PROJECT_NAME}
    SHARED
    controller/multi_controller.cpp
)


target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
    ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "MANIPULATOR_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}.xml)


install(
  DIRECTORY controller/include/
  DESTINATION include/
)


install(
  PROGRAMS  ${PROJECT_NAME}/error_plotter.py
            ${PROJECT_NAME}/send_pose.py
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()