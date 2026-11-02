set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_CROSS_COMPILE "/usr/bin/aarch64-linux-gnu-")
set(CMAKE_C_COMPILER          ${CMAKE_CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER        ${CMAKE_CROSS_COMPILE}g++)
set(CMAKE_FC_COMPILER         ${CMAKE_CROSS_COMPILE}gfortran)
set(CMAKE_Fortran_COMPILER    ${CMAKE_CROSS_COMPILE}gfortran)

# 加入打印，减少编译时提示变量未使用的频繁打印
message(STATUS "Using CMAKE_CUSTOM_CROSS: ${CMAKE_CUSTOM_CROSS}.")

# 指定 sysroot 路径
set(CMAKE_SYSROOT /opt/rootfs_2204_aarch64)

# 确保搜索路径指向 sysroot
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
if(DEFINED ENV{HL_MOTION_PREFIX} AND EXISTS "$ENV{HL_MOTION_PREFIX}")
  list(APPEND CMAKE_FIND_ROOT_PATH "$ENV{HL_MOTION_PREFIX}")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HL_MOTION_PREFIX}")
endif()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_PREFIX_PATH 
  ${CMAKE_PREFIX_PATH}
  ${CMAKE_SYSROOT}/opt/ros/humble
  ${CMAKE_SYSROOT}/opt/ros/humble/share
  ${CMAKE_SYSROOT}/opt/ros/humble/share/rmw_fastrtps_cpp/cmake/
)

set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH}
  ${CMAKE_SYSROOT}/opt/ros/humble/share
  ${CMAKE_SYSROOT}/opt/ros/humble/share/rmw_fastrtps_cpp/cmake/
)


include_directories(
    ${CMAKE_SYSROOT}/usr/include/
    ${CMAKE_SYSROOT}/usr/include/python3.10
    ${CMAKE_SYSROOT}/usr/include/eigen3
)

link_directories(
    ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
    ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/python3.10.so
)

# 增加编译宏定义给src文件使用
add_compile_options(-Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-deprecated-declarations -Wno-reorder -Wno-attributes)
add_compile_options(-Wno-maybe-uninitialized -Wno-return-type)
add_compile_options(-Wno-psabi)  # pinocchio warning 提醒忽略
