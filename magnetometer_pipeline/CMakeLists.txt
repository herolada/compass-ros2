# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(magnetometer_pipeline)
set (CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(compass_utils REQUIRED)
find_package(compass_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tl-expected REQUIRED)

# LIBRARY 1
add_library(magnetometer_bias_remover SHARED
  src/bias_remover.cpp
)
ament_target_dependencies(magnetometer_bias_remover
  compass_conversions
  compass_utils
  rclcpp
  sensor_msgs
  tf2_eigen
)
target_link_libraries(magnetometer_bias_remover tl::expected)

target_include_directories(magnetometer_bias_remover
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "/usr/include/eigen3"
)
ament_export_targets(export_magnetometer_bias_remover HAS_LIBRARY_TARGET)
# ament_export_dependencies( 
#   sensor_msgs
#   rclcpp
#   tf2_eigen
# )
ament_export_dependencies( 
  compass_conversions
  compass_utils
  rclcpp
  sensor_msgs
  tf2_eigen
)

# LIBRARY 2
add_library(magnetometer_bias_remover_filter SHARED
  src/message_filter.cpp
)
ament_target_dependencies(magnetometer_bias_remover_filter
  geometry_msgs
  message_filters
  rclcpp
  rcutils
  sensor_msgs
)

target_link_libraries(magnetometer_bias_remover_filter magnetometer_bias_remover tl::expected)

target_include_directories(magnetometer_bias_remover_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_magnetometer_bias_remover_filter HAS_LIBRARY_TARGET)
# ament_export_dependencies( 
#   message_filters
#   sensor_msgs
# )
ament_export_dependencies( 
  geometry_msgs
  message_filters
  rclcpp
  rcutils
  sensor_msgs
)

# LIBRARY 3
add_library(magnetometer_bias_remover_nodelet_lib SHARED
  nodelets/magnetometer_bias_remover_nodelet.cpp
)
ament_target_dependencies(magnetometer_bias_remover_nodelet_lib
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(magnetometer_bias_remover_nodelet_lib magnetometer_bias_remover_filter tl::expected)

target_include_directories(magnetometer_bias_remover_nodelet_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_magnetometer_bias_remover_nodelet_lib HAS_LIBRARY_TARGET)
# ament_export_dependencies( 
#   message_filters
#   rclcpp
#   sensor_msgs
# )
ament_export_dependencies( 
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(magnetometer_bias_remover_nodelet_lib "magnetometer_pipeline::MagnetometerBiasRemoverNodelet")

# INSTALL LIBRARIES
install(
    DIRECTORY include/
    DESTINATION include
    )
install(TARGETS
  magnetometer_bias_remover
  EXPORT
  export_magnetometer_bias_remover
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  magnetometer_bias_remover_filter
  EXPORT
  export_magnetometer_bias_remover_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  magnetometer_bias_remover_nodelet_lib
  EXPORT
  export_magnetometer_bias_remover_nodelet_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
add_executable(magnetometer_bias_remover_nodelet nodelets/magnetometer_bias_remover_nodelet.cpp)

ament_target_dependencies(magnetometer_bias_remover_nodelet
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
)

target_link_libraries(magnetometer_bias_remover_nodelet magnetometer_bias_remover_nodelet_lib magnetometer_bias_remover_filter)

target_include_directories(magnetometer_bias_remover_nodelet PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(magnetometer_bias_remover_nodelet PUBLIC c_std_99 cxx_std_23)

install(TARGETS magnetometer_bias_remover_nodelet
    DESTINATION lib/${PROJECT_NAME})

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

### PYTHON NODE

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/magnetometer_bias_observer.py
  DESTINATION lib/${PROJECT_NAME}
)

### PYTHON NODE END

install(FILES nodelets.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/launch${PROJECT_NAME})


if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_magnetometer_bias_remover_nodelet test/test_bias_remover_nodelet.cpp)
    target_link_libraries(test_magnetometer_bias_remover_nodelet magnetometer_bias_remover_nodelet_lib)
    target_include_directories(test_magnetometer_bias_remover_nodelet
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_magnetometer_bias_remover test/test_bias_remover.cpp)
    ament_target_dependencies(test_magnetometer_bias_remover
    )
    target_link_libraries(test_magnetometer_bias_remover magnetometer_bias_remover tl::expected)
    target_include_directories(test_magnetometer_bias_remover
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_magnetometer_bias_remover_filter test/test_message_filter.cpp)
    target_link_libraries(test_magnetometer_bias_remover_filter magnetometer_bias_remover_filter magnetometer_bias_remover)
    target_include_directories(test_magnetometer_bias_remover_filter
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