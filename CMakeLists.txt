cmake_minimum_required(VERSION 3.16)
project("rl_controller_runner")

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-fPIC)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# 设置ONNX Runtime的头文件目录
# 根据系统架构映射到正确的目录名
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
    set(ONNX_ARCH "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(ONNX_ARCH "aarch64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(ONNXRUNTIME_LIB "${ROOT_DIR}/third_party/onnxruntime-${ONNX_ARCH}/lib")
set(ONNXRUNTIME_INCLUDE_DIRS "${ROOT_DIR}/third_party/onnxruntime-${ONNX_ARCH}/include")

find_library(ONNXRUNTIME_LIBRARIES NAMES onnxruntime PATHS ${ONNXRUNTIME_LIB} NO_DEFAULT_PATH)
if(NOT ONNXRUNTIME_LIBRARIES)
    set(ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_LIB}/libonnxruntime.so)
    if(NOT EXISTS ${ONNXRUNTIME_LIBRARIES})
        message(FATAL_ERROR "ONNX Runtime library not found at ${ONNXRUNTIME_LIBRARIES}")
    endif()
endif()

# 可选：添加其他provider库（如果存在）
find_library(ONNXRUNTIME_PROVIDERS_SHARED NAMES onnxruntime_providers_shared PATHS ${ONNXRUNTIME_LIB} NO_DEFAULT_PATH)
if(ONNXRUNTIME_PROVIDERS_SHARED)
    list(APPEND ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_PROVIDERS_SHARED})
endif()

# 可选：添加CUDA provider（仅在aarch64上可能存在）
find_library(ONNXRUNTIME_PROVIDERS_CUDA NAMES onnxruntime_providers_cuda PATHS ${ONNXRUNTIME_LIB} NO_DEFAULT_PATH)
if(ONNXRUNTIME_PROVIDERS_CUDA)
    list(APPEND ONNXRUNTIME_LIBRARIES ${ONNXRUNTIME_PROVIDERS_CUDA})
    message(STATUS "Found CUDA provider: ${ONNXRUNTIME_PROVIDERS_CUDA}")
endif()

# 安装ONNX Runtime库文件（包括符号链接和依赖库）
if(EXISTS ${ONNXRUNTIME_LIB})
    # 获取所有.so文件（包括符号链接）
    file(GLOB ONNX_SO_FILES "${ONNXRUNTIME_LIB}/libonnxruntime*.so*")
    if(ONNX_SO_FILES)
        install(FILES ${ONNX_SO_FILES}
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
        message(STATUS "Found ONNX Runtime libraries: ${ONNX_SO_FILES}")
    else()
        message(WARNING "No ONNX Runtime .so files found in ${ONNXRUNTIME_LIB}")
    endif()
else()
    message(WARNING "ONNX Runtime library directory not found at ${ONNXRUNTIME_LIB}")
endif()


# 第三方库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(crb_ros_msg REQUIRED)

# Keep ROS message type-support libraries discoverable when the installed
# runner is launched directly from /workspace/rl_controller/bin.
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

file(GLOB CRB_ROS_RUNTIME_LIBS "${crb_ros_msg_DIR}/../../../lib/libcrb_ros_msg*.so*")
if(CRB_ROS_RUNTIME_LIBS)
    install(FILES ${CRB_ROS_RUNTIME_LIBS} DESTINATION lib)
    message(STATUS "Bundling crb_ros_msg runtime libs: ${CRB_ROS_RUNTIME_LIBS}")
endif()


# 添加子目录
add_subdirectory(common)
add_subdirectory(rl_controller)


# 源码 - 只包含主程序入口，避免递归包含CMake临时文件和子目录源码
add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} 
    PUBLIC
        ${ONNXRUNTIME_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/hl_common
        ${CMAKE_CURRENT_SOURCE_DIR}/rl_controller
    PRIVATE
       ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    ${ONNXRUNTIME_LIBRARIES}
    yaml-cpp
    rl_controller
    rl_common
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    diagnostic_msgs
    crb_ros_msg
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_WITH_INSTALL_RPATH FALSE
)

file(GLOB_RECURSE PUBLIC_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
)


install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include
)


install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
        DESTINATION ${CMAKE_INSTALL_PREFIX}
)


install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup.sh
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup.bash
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup.zsh
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
)
