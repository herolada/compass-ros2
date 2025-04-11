cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(magnetic_model)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(compass_interfaces REQUIRED)
find_package(compass_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tl-expected REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# The most common use case is to merge everything you need to export
# into the same shared library called ${PROJECT_NAME}.
add_library(${PROJECT_NAME} SHARED
    src/magnetic_model_manager.cpp
    src/magnetic_model.cpp
    )

# all includes
ament_target_dependencies(${PROJECT_NAME}
    angles
    compass_interfaces
    compass_utils
    geometry_msgs
    rclcpp
    sensor_msgs
    GeographicLib
    ament_index_cpp
)

target_link_libraries(${PROJECT_NAME} tl::expected)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# hpp includes
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies( 
    geometry_msgs
    rclcpp
    sensor_msgs
)

install(
    DIRECTORY include/
    DESTINATION include
    )
    
install(
    DIRECTORY share/data
    DESTINATION share/${PROJECT_NAME}
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

if(BUILD_TESTING)

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
    target_include_directories(test_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE -DTEST_DATA_DIR="${CMAKE_SOURCE_DIR}/share/data/magnetic")
    #ament_target_dependencies(test_${PROJECT_NAME}
    #    rclcpp
    #    compass_interfaces
    #)


    #find_package(ament_cmake_gtest REQUIRED)
    #ament_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
    #target_include_directories(test_${PROJECT_NAME} PUBLIC
    #    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #    $<INSTALL_INTERFACE:include>
    #)
    #target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
    #target_compile_definitions(test_${PROJECT_NAME} PRIVATE -DTEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data/magnetic")
    
    #find_package(ament_lint_auto REQUIRED)
    #set(ament_cmake_copyright_FOUND TRUE)
    #set(ament_cmake_cpplint_FOUND TRUE)
    #ament_lint_auto_find_test_dependencies()
endif()

ament_package()