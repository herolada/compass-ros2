cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 23)
project(test)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(magnetometer_pipeline REQUIRED)
find_package(compass_conversions REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(test_pckg SHARED
  src/test.cpp
)
ament_target_dependencies(test_pckg
  compass_conversions
)

target_include_directories(test_pckg
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(test_pckg
compass_conversions::compass_topic_names
)

ament_export_targets(export_test_pckg HAS_LIBRARY_TARGET)
ament_export_dependencies(
  compass_conversions
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS test_pckg
  EXPORT export_test_pckg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
  target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  # ament_target_dependencies(${PROJECT_NAME}_tutorial_test
  #   std_msgs
  # )
  # target_link_libraries(${PROJECT_NAME}_tutorial_test name_of_local_library)
  target_link_libraries(${PROJECT_NAME}_tutorial_test compass_conversions::compass_topic_names
)
endif()

ament_package()
