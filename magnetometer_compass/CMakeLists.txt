# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(magnetometer_compass)
set (CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(compass_conversions REQUIRED)
find_package(compass_interfaces REQUIRED)
find_package(compass_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(imu_transformer REQUIRED)
find_package(magnetometer_pipeline REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tl-expected REQUIRED)


# LIBRARY 1
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  angles
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(${PROJECT_NAME} tl::expected)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/usr/include/eigen3"
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies( 
#   compass_interfaces
#   magnetometer_pipeline
#   sensor_msgs
#   rclcpp
#   tf2
# )
ament_export_dependencies( 
  angles
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# LIBRARY 2
add_library(${PROJECT_NAME}_nodelet_lib SHARED
  nodelets/${PROJECT_NAME}_nodelet.cpp
)
ament_target_dependencies(${PROJECT_NAME}_nodelet_lib
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
pluginlib_export_plugin_description_file(rclcpp plugins.xml)

target_link_libraries(${PROJECT_NAME}_nodelet_lib ${PROJECT_NAME} tl::expected)

target_include_directories(${PROJECT_NAME}_nodelet_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/usr/include/eigen3"
)
ament_export_targets(export_${PROJECT_NAME}_nodelet_lib HAS_LIBRARY_TARGET)
# ament_export_dependencies( 
#   compass_interfaces
#   imu_transformer
#   magnetometer_pipeline
#   sensor_msgs
#   rclcpp
#   tf2
# )
ament_export_dependencies( 
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(${PROJECT_NAME}_nodelet_lib "magnetometer_compass::MagnetometerCompassNodelet")


# LIBRARY 3
add_library(visualize_azimuth_nodelet_lib SHARED
  nodelets/visualize_azimuth_nodelet.cpp
)
ament_target_dependencies(visualize_azimuth_nodelet_lib
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(visualize_azimuth_nodelet_lib tl::expected)

target_include_directories(visualize_azimuth_nodelet_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_visualize_azimuth_nodelet_lib HAS_LIBRARY_TARGET)
# ament_export_dependencies( 
#   compass_interfaces
#   magnetometer_pipeline
#   sensor_msgs
#   rclcpp
#   tf2
# )
ament_export_dependencies( 
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(visualize_azimuth_nodelet_lib "magnetometer_compass::VisualizeAzimuthNodelet")


# INSTALL LIBRARIES
install(
    DIRECTORY include/
    DESTINATION include
    )
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${PROJECT_NAME}_nodelet_lib
  EXPORT export_${PROJECT_NAME}_nodelet_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  visualize_azimuth_nodelet_lib
  EXPORT export_visualize_azimuth_nodelet_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# Do we need this???:
# https://github.com/ros2/demos/blob/23b3bb5807223673cfd3f5e5652e8b0fe2568d7d/composition/CMakeLists.txt#L77
# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
#if(NOT WIN32)
#  ament_environment_hooks(
#    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
#endif()


############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
add_executable(${PROJECT_NAME}_nodelet nodelets/${PROJECT_NAME}_nodelet.cpp)

ament_target_dependencies(${PROJECT_NAME}_nodelet
  angles
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_nodelet_lib ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}_nodelet PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_nodelet PUBLIC c_std_99 cxx_std_23)

install(TARGETS ${PROJECT_NAME}_nodelet
    DESTINATION lib/${PROJECT_NAME})

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

add_executable(visualize_azimuth_nodelet nodelets/visualize_azimuth_nodelet.cpp)

ament_target_dependencies(visualize_azimuth_nodelet
  angles
  compass_conversions
  compass_interfaces
  compass_utils
  geometry_msgs
  imu_transformer
  magnetometer_pipeline
  message_filters
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(visualize_azimuth_nodelet visualize_azimuth_nodelet_lib)

target_include_directories(visualize_azimuth_nodelet PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(visualize_azimuth_nodelet PUBLIC c_std_99 cxx_std_23)

install(TARGETS visualize_azimuth_nodelet
    DESTINATION lib/${PROJECT_NAME})

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

install(FILES nodelets.xml
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(imu_transformer REQUIRED)
    find_package(tf2_sensor_msgs REQUIRED)

    ament_add_gtest(test_magnetometer_compass_nodelet test/test_magnetometer_compass_nodelet.cpp)
    ament_target_dependencies(test_magnetometer_compass_nodelet
      builtin_interfaces
      imu_transformer
      tf2_sensor_msgs
    )
    target_link_libraries(test_magnetometer_compass_nodelet magnetometer_compass_nodelet_lib)
    target_include_directories(test_magnetometer_compass_nodelet
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_magnetometer_compass test/test_magnetometer_compass.cpp)
    target_link_libraries(test_magnetometer_compass magnetometer_compass)
    target_include_directories(test_magnetometer_compass
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_visualize_azimuth_nodelet test/test_visualize_azimuth_nodelet.cpp)
    target_link_libraries(test_visualize_azimuth_nodelet visualize_azimuth_nodelet_lib)
    target_include_directories(test_visualize_azimuth_nodelet
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()