cmake_minimum_required(VERSION 3.20)
set (CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(compass_utils)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(compass_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tl-expected REQUIRED)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# The most common use case is to merge everything you need to export
# into the same shared library called ${PROJECT_NAME}.
add_library(${PROJECT_NAME} SHARED
    src/string_utils.cpp
    src/time_utils.cpp
    src/rate_limiter.cpp
    src/tf2_utils.cpp
)

# all includes
ament_target_dependencies(${PROJECT_NAME}
    compass_interfaces
    geometry_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tl-expected
    )

target_link_libraries(${PROJECT_NAME} tl::expected)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../tl_expected/include>
    $<INSTALL_INTERFACE:include>)

# hpp includes
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies( 
    compass_interfaces
    rclcpp
    tf2
    tf2_geometry_msgs
    # tl-expected
    )
# ament_export_dependencies(
#     compass_interfaces
#     geometry_msgs
#     rclcpp
#     tf2
#     tf2_geometry_msgs
#     tl-expected
#     )

install(
    DIRECTORY include/
    DESTINATION include)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(std_msgs REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
    target_include_directories(test_${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
    target_link_libraries(test_rate_limiter ${PROJECT_NAME})
    target_include_directories(test_rate_limiter
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    # set_tests_properties(test_rate_limiter PROPERTIES ENVIRONMENT "GTEST_CAPTURE=0")

    ament_add_gtest(test_string_utils test/test_string_utils.cpp)
    target_link_libraries(test_string_utils ${PROJECT_NAME})
    target_include_directories(test_string_utils
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_tf2_utils test/test_tf2_utils.cpp)
    target_link_libraries(test_tf2_utils ${PROJECT_NAME})
    target_include_directories(test_tf2_utils
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    ament_add_gtest(test_time_utils test/test_time_utils.cpp)
    target_link_libraries(test_time_utils ${PROJECT_NAME})
    target_include_directories(test_time_utils
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
        
    #find_package(ament_lint_auto REQUIRED)
    #set(ament_cmake_copyright_FOUND TRUE)
    #set(ament_cmake_cpplint_FOUND TRUE)
    #ament_lint_auto_find_test_dependencies()
endif()


ament_package()