# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20)
project(magnetometer_compass)
set (CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(angles REQUIRED)
find_package(compass_conversions REQUIRED)
find_package(compass_interfaces REQUIRED)
find_package(compass_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(imu_transformer REQUIRED)
find_package(magnetometer_pipeline REQUIRED)
find_package(message_filters REQUIRED)
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
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  compass_interfaces
  sensor_msgs
  rclcpp
  tf2
)

# LIBRARY 2
add_library(${PROJECT_NAME}_nodelet SHARED
  nodelets/${PROJECT_NAME}_nodelet.cpp
)
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
target_link_libraries(${PROJECT_NAME}_nodelet tl::expected)

target_include_directories(${PROJECT_NAME}_nodelet
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_${PROJECT_NAME}_nodelet HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  compass_interfaces
  sensor_msgs
  rclcpp
  tf2
)


# LIBRARY 3
add_library(visualize_azimuth_nodelet SHARED
  nodelets/visualize_azimuth_nodelet.cpp
)
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
target_link_libraries(visualize_azimuth_nodelet tl::expected)

target_include_directories(visualize_azimuth_nodelet
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_visualize_azimuth_nodelet HAS_LIBRARY_TARGET)
ament_export_dependencies( 
  compass_interfaces
  sensor_msgs
  rclcpp
  tf2
)


# INSTALL LIBRARIES
install(
    DIRECTORY include/
    DESTINATION include
    )
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_nodelet
  visualize_azimuth_nodelet
  EXPORT export_${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}_nodelet
  EXPORT export_visualize_azimuth_nodelet
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
# add_executable(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_nodelet.cpp)

# ament_target_dependencies(${PROJECT_NAME}_nodelet
#   angles
#   compass_conversions
#   compass_interfaces
#   compass_utils
#   geometry_msgs
#   imu_transformer
#   magnetometer_pipeline
#   message_filters
#   rclcpp
#   sensor_msgs
#   tf2
#   tf2_geometry_msgs
#   tf2_ros
# )

# target_include_directories(${PROJECT_NAME}_nodelet PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

# target_compile_features(${PROJECT_NAME}_nodelet PUBLIC c_std_99 cxx_std_23)

# install(TARGETS ${PROJECT_NAME}_nodelet
#     DESTINATION lib/${PROJECT_NAME})

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

# add_executable(visualize_azimuth_nodelet visualize_azimuth_nodelet.cpp)

# ament_target_dependencies(visualize_azimuth_nodelet
#   angles
#   compass_conversions
#   compass_interfaces
#   compass_utils
#   geometry_msgs
#   imu_transformer
#   magnetometer_pipeline
#   message_filters
#   rclcpp
#   sensor_msgs
#   tf2
#   tf2_geometry_msgs
#   tf2_ros
# )

# target_include_directories(visualize_azimuth_nodelet PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

# target_compile_features(visualize_azimuth_nodelet PUBLIC c_std_99 cxx_std_23)

# install(TARGETS visualize_azimuth_nodelet
#     DESTINATION lib/${PROJECT_NAME})

# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

install(FILES nodelets.xml
DESTINATION share/${PROJECT_NAME}
)

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

#   roslint_add_test()

#   roslaunch_add_file_check(launch/compass.launch)

#   catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
#   target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

#   add_rostest_gtest(test_${PROJECT_NAME}_nodelet test/test_${PROJECT_NAME}_nodelet.test test/test_${PROJECT_NAME}_nodelet.cpp)
#   target_link_libraries(test_${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME}_nodelet)
#   roslaunch_add_file_check(test/test_${PROJECT_NAME}_nodelet.test USE_TEST_DEPENDENCIES)
#   if(CMAKE_VERSION VERSION_LESS "3.13.0")
#     set_property(TARGET test_${PROJECT_NAME}_nodelet APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-as-needed")
#   else()
#     #catkin_lint: ignore_once cmake_old
#     target_link_options(test_${PROJECT_NAME}_nodelet PUBLIC "LINKER:--no-as-needed")
#   endif()

#   add_rostest_gtest(test_visualize_azimuth_nodelet test/test_visualize_azimuth_nodelet.test test/test_visualize_azimuth_nodelet.cpp)
#   target_link_libraries(test_visualize_azimuth_nodelet ${catkin_LIBRARIES})
#   roslaunch_add_file_check(test/test_visualize_azimuth_nodelet.test USE_TEST_DEPENDENCIES)
# endif()
ament_package()