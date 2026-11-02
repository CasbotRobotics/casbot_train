cmake_minimum_required(VERSION 3.16)
project(rl_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hpp-fcl REQUIRED)

find_package(yaml-cpp REQUIRED)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/joint_info SRCS)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/log SRCS)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/message SRCS)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/utils SRCS)

add_library(${PROJECT_NAME} SHARED ${SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>
)


target_link_libraries(${PROJECT_NAME} PUBLIC
    hpp-fcl::hpp-fcl
    yaml-cpp
    fmt
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    geometry_msgs
    sensor_msgs
)

install(TARGETS ${PROJECT_NAME}
    EXPORT  ${PROJECT_NAME}Targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs glog)

ament_package()