# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(compass_conversions)
set (CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(compass_interfaces REQUIRED)
find_package(compass_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(magnetic_model REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tl-expected REQUIRED)
find_package(topic_tools REQUIRED)

# Ubuntu libgeographic-dev package installs into non-standard location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# LIBRARY 1
add_library(compass_topic_names SHARED
  src/topic_names.cpp
)
ament_target_dependencies(compass_topic_names
  compass_interfaces
  compass_utils
  geometry_msgs
  sensor_msgs
  tl-expected
)
target_link_libraries(compass_topic_names tl::expected)

target_include_directories(compass_topic_names
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_compass_topic_names HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  compass_interfaces
  compass_utils
  geometry_msgs
  sensor_msgs
  tl-expected
)

# LIBRARY 2
add_library(compass_converter SHARED
  src/compass_converter.cpp
)
ament_target_dependencies(compass_converter
  angles
  compass_interfaces
  compass_utils
  GeographicLib
  geometry_msgs
  magnetic_model
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tl-expected
)
target_link_libraries(compass_converter compass_topic_names tl::expected)

target_include_directories(compass_converter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_compass_converter HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  compass_utils
  magnetic_model
  angles
  compass_interfaces
  std_msgs
  rclcpp
  geometry_msgs
  sensor_msgs
  tl-expected
)

# LIBRARY 3
add_library(compass_message_filter SHARED
  src/message_filter.cpp
)
ament_target_dependencies(compass_message_filter
  angles
  compass_interfaces
  compass_utils
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tl-expected
)
target_link_libraries(compass_message_filter compass_converter tl::expected)

target_include_directories(compass_message_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_compass_message_filter HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  compass_interfaces
  compass_utils
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tl-expected
)

# LIBRARY 4
add_library(tf2_compass_msgs SHARED
  src/tf2_compass_msgs.cpp
)
ament_target_dependencies(tf2_compass_msgs
  angles
  compass_utils
  compass_interfaces
  geometry_msgs
  rclcpp
  tf2
  tl-expected
)
target_link_libraries(tf2_compass_msgs tl::expected)

target_include_directories(tf2_compass_msgs
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_tf2_compass_msgs HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  compass_interfaces
  compass_utils
  rclcpp
  geometry_msgs
  tf2
  tl-expected
)

# LIBRARY 5
add_library(compass_transformer_nodelet_lib SHARED
  nodelets/compass_transformer.cpp
)
ament_target_dependencies(compass_transformer_nodelet_lib
  angles
  compass_interfaces
  compass_utils
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(compass_transformer_nodelet_lib compass_topic_names compass_converter compass_message_filter tf2_compass_msgs tl::expected)

target_include_directories(compass_transformer_nodelet_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_compass_transformer_nodelet_lib HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  angles
  compass_interfaces
  compass_utils
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
rclcpp_components_register_nodes(compass_transformer_nodelet_lib "compass_conversions::CompassTransformerNodelet")
#https://github.com/ros2/demos/blob/jazzy/composition/CMakeLists.txt
#set(node_plugins "${node_plugins}compass_conversions::CompassTransformerNodelet;$<TARGET_FILE:compass_transformer_nodelet>\n")

# INSTALL LIBRARIES
install(
    DIRECTORY include/
    DESTINATION include
    )
install(TARGETS
  compass_topic_names
  EXPORT
  export_compass_topic_names
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(TARGETS
  compass_converter

  EXPORT
  export_compass_converter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(TARGETS
  compass_message_filter
  EXPORT
  export_compass_message_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(TARGETS
  tf2_compass_msgs
  EXPORT
  export_tf2_compass_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  install(TARGETS
  compass_transformer_nodelet_lib
  EXPORT
  export_compass_transformer_nodelet_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset

add_executable(compass_transformer_nodelet nodelets/compass_transformer.cpp)

ament_target_dependencies(compass_transformer_nodelet
  angles
  compass_interfaces
  compass_utils
  geometry_msgs
  magnetic_model
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  topic_tools
)
target_link_libraries(compass_transformer_nodelet compass_transformer_nodelet_lib compass_topic_names compass_converter compass_message_filter tf2_compass_msgs)

target_include_directories(compass_transformer_nodelet PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(compass_transformer_nodelet PUBLIC c_std_99 cxx_std_23)

install(TARGETS compass_transformer_nodelet
    DESTINATION lib/${PROJECT_NAME})

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

install(FILES nodelets.xml
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_compass_converter test/test_compass_converter.cpp)
    target_link_libraries(test_compass_converter compass_converter compass_topic_names)
    target_include_directories(test_compass_converter
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    set_tests_properties(test_compass_converter PROPERTIES ENVIRONMENT "GTEST_CAPTURE=0")

    ament_add_gtest(test_compass_transformer_nodelet test/test_compass_transformer_nodelet.cpp)
    target_link_libraries(test_compass_transformer_nodelet
      compass_transformer_nodelet_lib
      compass_converter
      compass_message_filter
      compass_topic_names 
      tf2_compass_msgs)
    target_include_directories(test_compass_transformer_nodelet
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_compass_message_filter test/test_message_filter.cpp)
    target_link_libraries(test_compass_message_filter
      compass_message_filter
      compass_converter
      compass_topic_names)
    target_include_directories(test_compass_message_filter
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    ament_add_gtest(test_tf2_compass_msgs test/test_tf2_compass_msgs.cpp)
    target_link_libraries(test_tf2_compass_msgs tf2_compass_msgs)
    target_include_directories(test_tf2_compass_msgs
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    ament_add_gtest(test_compass_topic_names test/test_topic_names.cpp)
    target_link_libraries(test_compass_topic_names compass_topic_names)
    target_include_directories(test_compass_topic_names
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