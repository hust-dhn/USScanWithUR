cmake_minimum_required(VERSION 3.10)
project(display_camera)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 设置Inuitive SDK路径（根据您的实际安装路径修改）
set(INUITIVE_SDK_DIR "/home/zzz/ros2_ws/src/my_ur10e_control/SDK_CAMERA/Linux/Linux/Inuitive/InuDev")
set(INUITIVE_LIB_DIR "/home/zzz/ros2_ws/src/my_ur10e_control/SDK_CAMERA/Linux/Linux/Inuitive/InuDev/lib")

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${INUITIVE_SDK_DIR}/include)

# 添加可执行文件
add_executable(display_camera displayInuRGB.cpp)

# 链接库
target_link_libraries(display_camera ${OpenCV_LIBS})
target_link_libraries(display_camera -L${INUITIVE_LIB_DIR} -linuitive_core)

# 设置可执行文件输出目录
set_target_properties(display_camera PROPERTIES
    /home/zzz/ros2_ws/src/my_ur10e_control/display_camera/result ${CMAKE_BINARY_DIR}/bin
)