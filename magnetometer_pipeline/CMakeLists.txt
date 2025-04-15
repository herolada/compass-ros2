# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(magnetometer_pipeline)
set (CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(compass_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tl-expected REQUIRED)

# LIBRARY 1
add_library(magnetometer_bias_remover SHARED
  src/bias_remover.cpp
)
ament_target_dependencies(magnetometer_bias_remover
  compass_utils
  rclcpp
  sensor_msgs
  tf2_eigen
  Eigen3
)
target_link_libraries(magnetometer_bias_remover tl::expected)

target_include_directories(magnetometer_bias_remover
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_magnetometer_bias_remover HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  sensor_msgs
  rclcpp
)

# LIBRARY 2
add_library(magnetometer_bias_remover_filter SHARED
  src/message_filter.cpp
)
ament_target_dependencies(magnetometer_bias_remover_filter
  geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  tf2_eigen
  Eigen3
)

target_include_directories(magnetometer_bias_remover_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_magnetometer_bias_remover_filter HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  message_filters
  sensor_msgs
)

# LIBRARY 3
add_library(magnetometer_bias_remover_nodelet SHARED
  nodelets/magnetometer_bias_remover_nodelet.cpp
)
ament_target_dependencies(magnetometer_bias_remover_nodelet
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_eigen
  Eigen3
)
target_link_libraries(magnetometer_bias_remover_nodelet tl::expected)

target_include_directories(magnetometer_bias_remover_nodelet
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_magnetometer_bias_remover_nodelet HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  message_filters
  sensor_msgs
  magnetometer_bias_remover
)

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
  magnetometer_bias_remover_nodelet
  EXPORT
  export_magnetometer_bias_remover_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
# add_executable(magnetometer_bias_remover_nodelet_node nodelets/magnetometer_bias_remover_nodelet.cpp)

# ament_target_dependencies(magnetometer_bias_remover_nodelet_node
#   compass_utils
#   geometry_msgs
#   message_filters
#   rclcpp
#   sensor_msgs
#   tf2_eigen
#   Eigen3
# )

# target_include_directories(magnetometer_bias_remover_nodelet_node PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

# target_compile_features(magnetometer_bias_remover_nodelet_node PUBLIC c_std_99 cxx_std_23)

# install(TARGETS magnetometer_bias_remover_nodelet_node
#     DESTINATION lib/${PROJECT_NAME})

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

install(FILES nodelets.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/launch${PROJECT_NAME})

# catkin_install_python(PROGRAMS nodes/magnetometer_bias_observer
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# if(CATKIN_ENABLE_TESTING)
#   find_package(roslint REQUIRED)
#   find_package(rostest REQUIRED)
#   find_package(roslaunch REQUIRED)

#   roslint_custom(catkin_lint "-W2" .)

#   # Roslint C++ - checks formatting and some other rules for C++ files
#   file(GLOB_RECURSE ROSLINT_FILES include/*.h src/*.cpp nodelets/*.cpp test/*.cpp)
#   set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
#     -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
#     -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
#   roslint_cpp(${ROSLINT_FILES})

#   # Run roslint on Python sources
#   file(GLOB_RECURSE PYTHON_FILES nodes/magnetometer_bias_observer)
#   roslint_python("${PYTHON_FILES}")

#   roslint_add_test()

#   roslaunch_add_file_check(launch/magnetometer_proc.launch)

#   catkin_add_gtest(test_magnetometer_bias_remover test/test_bias_remover.cpp)
#   target_link_libraries(test_magnetometer_bias_remover magnetometer_bias_remover)

#   catkin_add_gtest(test_magnetometer_bias_remover_filter test/test_message_filter.cpp)
#   target_link_libraries(test_magnetometer_bias_remover_filter magnetometer_bias_remover_filter)

#   add_rostest_gtest(test_magnetometer_bias_remover_nodelet
#     test/test_bias_remover_nodelet.test test/test_bias_remover_nodelet.cpp)
#   target_link_libraries(test_magnetometer_bias_remover_nodelet ${catkin_LIBRARIES})
#   roslaunch_add_file_check(test/test_bias_remover_nodelet.test USE_TEST_DEPENDENCIES)
# endif()
