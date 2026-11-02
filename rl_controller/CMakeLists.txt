cmake_minimum_required(VERSION 3.16)
project("rl_controller")


# 第三方库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

# 源码
file(GLOB_RECURSE SRC_FILES
    CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(${PROJECT_NAME} 
    PUBLIC
        ${ONNXRUNTIME_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../common
        ${CMAKE_CURRENT_SOURCE_DIR}/../
        ${EIGEN3_INCLUDE_DIR}
    PRIVATE
       ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    ${ONNXRUNTIME_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    rl_common
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    diagnostic_msgs
    spdlog
)

file(GLOB_RECURSE PUBLIC_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    PUBLIC_HEADER "${PUBLIC_HEADERS}"
)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include/controller
)
