cmake_minimum_required(VERSION 3.16)


#set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "info" FORCE)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "info" FORCE)
#set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "info" FORCE)
#set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "info" FORCE)
set(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE})
message("========== 编译模式 CMAKE_BUILD_TYPE         : ${CMAKE_BUILD_TYPE}")
message("========== 编译模式 CMAKE_CONFIGURATION_TYPES: ${CMAKE_CONFIGURATION_TYPES}")


project(qt_vtk_pcl_registration)


if (UNIX)
    message("==========Found CMAKE_C_COMPILER  :${CMAKE_C_COMPILER}")      #/usr/bin/cc
    message("==========Found CMAKE_CXX_COMPILER:${CMAKE_CXX_COMPILER}")    #/usr/bin/c++
    message("==========Found CMAKE_LINKER      :${CMAKE_LINKER}")          #/usr/bin/ld
    message("==========Found CMAKE_MAKE_PROGRAM:${CMAKE_MAKE_PROGRAM}")    #/usr/bin/make
elseif (WIN32)
    if (MSVC)
        message("==========Found CMAKE_C_COMPILER  :${CMAKE_C_COMPILER}")      #cl.exe
        message("==========Found CMAKE_CXX_COMPILER:${CMAKE_CXX_COMPILER}")    #cl.exe
        message("==========Found CMAKE_LINKER      :${CMAKE_LINKER}")          #link.exe
        message("==========Found CMAKE_MAKE_PROGRAM:${CMAKE_MAKE_PROGRAM}")    #MSBuild.exe nmake.exe
    endif ()
endif ()


function(get_ubuntu_version etc_lsb_release ubuntu_version)
    # 检查是否为Ubuntu系统
    if(EXISTS ${etc_lsb_release})
        file(STRINGS ${etc_lsb_release} lsb_release_contents)
        foreach(line IN LISTS lsb_release_contents)
            if(line MATCHES "DISTRIB_ID=([^;]+)")
                set(distrib_id ${CMAKE_MATCH_1})
            elseif(line MATCHES "DISTRIB_RELEASE=([^;]+)")
                set(distrib_release ${CMAKE_MATCH_1})
            endif()
        endforeach()

        set(${ubuntu_version} ${distrib_id}_${distrib_release} PARENT_SCOPE)

        # 检查发行版ID是否为Ubuntu
        if(distrib_id STREQUAL "Ubuntu")
            # 检查版本是否为16.04
            if(${distrib_id}_${distrib_release} STREQUAL "Ubuntu_16.04")
                message("==============================System is Ubuntu_16.04")
                # 在这里添加配置或者逻辑
            elseif(${distrib_id}_${distrib_release} STREQUAL "Ubuntu_18.04")
                message("==============================System is Ubuntu_18.04")
                # 在这里添加配置或者逻辑
            elseif(${distrib_id}_${distrib_release} STREQUAL "Ubuntu_20.04")
                message("==============================System is Ubuntu_20.04")
                # 在这里添加配置或者逻辑
            else()
                message("==============================System is Ubuntu but not Ubuntu_16.04 Ubuntu_18.04 Ubuntu_20.04")
            endif()
        else()
            message("==============================System is not Ubuntu")
        endif()
    else()
        message("==============================Unable to detect system information")
    endif()
endfunction()


get_ubuntu_version("/etc/lsb-release" ubuntu_version)
message("==============================ubuntu_version:${ubuntu_version}")


if (UNIX)
    set(CMAKE_CXX_STANDARD 11)
    #set(CMAKE_CXX_STANDARD 14)
    #set(CMAKE_CXX_STANDARD 17)
endif (UNIX)

if (WIN32)
#    set(CMAKE_CXX_STANDARD 11)
    #set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD 17)

    #############"解决代码中有中文注释,编译时报错的问题"
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -source-charset:utf-8")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -source-charset:utf-8")
endif (WIN32)


#====================================================================================
#====================================================================================

message("==========Found PROJECT_NAME      :${PROJECT_NAME}")                 # Robot_modules_cpp
message("==========Found PROJECT_SOURCE_DIR:${PROJECT_SOURCE_DIR}")           # /home/liuhao/CLionProjects/Robot_modules_cpp
message("==========Found PROJECT_BINARY_DIR:${PROJECT_BINARY_DIR}")           # /home/liuhao/CLionProjects/Robot_modules_cpp/cmake-build-debug

#message("==========Found ${PROJECT_NAME}_SOURCE_DIR:${${PROJECT_NAME}_SOURCE_DIR}")   # /home/liuhao/CLionProjects/Robot_modules_cpp
#message("==========Found ${PROJECT_NAME}_BINARY_DIR=:${${PROJECT_NAME}_BINARY_DIR}")   # /home/liuhao/CLionProjects/Robot_modules_cpp/cmake-build-debug
#
#message("==========Found ==========Found CMAKE_PROJECT_NAME:${CMAKE_PROJECT_NAME}")           # Robot_modules_cpp
#message("==========Found ==========Found CMAKE_SOURCE_DIR  :${CMAKE_SOURCE_DIR}")             # /home/liuhao/CLionProjects/Robot_modules_cpp
#message("==========Found ==========Found CMAKE_BINARY_DIR  :${CMAKE_BINARY_DIR}")             # /home/liuhao/CLionProjects/Robot_modules_cpp/cmake-build-debug
#
## CMAKE_CURRENT_SOURCE_DIR只会看CMakeLists.txt在哪里,不会看<PackageName>Config.cmake或Find<PackageName>.cmake在哪里
#message("==========Found CMAKE_CURRENT_SOURCE_DIR:${CMAKE_CURRENT_SOURCE_DIR}")     # /home/liuhao/CLionProjects/Robot_modules_cpp
#message("==========Found CMAKE_CURRENT_BINARY_DIR:${CMAKE_CURRENT_BINARY_DIR}")     # /home/liuhao/CLionProjects/Robot_modules_cpp/cmake-build-debug

#====================================================================================
#====================================================================================

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# 若只是sleep_for时间休眠,CMakeLists不需要加这个设置
# 若是std::thread开启线程，CMakeLists就需要加这个设置
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#====================================================================================
#====================================================================================

# 使CMAKE_BUILD_TYPE(看clion的配置是Debug)变成小写debug,并赋值给cmake_build_type
string(TOLOWER ${CMAKE_BUILD_TYPE} cmake_build_type)    # cmake_build_type是我自己定义的
# 设置输出根目录为build/debug
#set(OUTPUT_DIRECTORY_ROOT ${PROJECT_SOURCE_DIR}/build/${cmake_build_type})

set(OUTPUT_DIRECTORY_ROOT .)

# 设置可执行程序输出到build/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin/" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib/" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib/" CACHE PATH "Archive directory" FORCE)

#====================================================================================
#====================================================================================

# 模块模式使用 CMAKE_MODULE_PATH,设置Find<PackageName>.cmake文件夹路径

# 配置模式使用 CMAKE_PREFIX_PATH,设置<PackageName>Config.cmake文件夹路径

set(3rdparty_PATH "/home/zzz/ros2_ws/src/3rdparty/")


#====================================================================================
#====================================================================================
set(boost_version 1.72.0)

# 使用正则表达式匹配所有数字
string(REGEX MATCHALL "[0-9]+" boost_version_major_minor ${boost_version})
#foreach(word IN LISTS boost_version_major_minor)
#    message("==================${word}")
#endforeach()
list(GET boost_version_major_minor 0 boost_version_major)
list(GET boost_version_major_minor 1 boost_version_minor)
list(GET boost_version_major_minor 2 boost_version_patch)
#message("==================${boost_version_major}")
#message("==================${boost_version_minor}")
#message("==================${boost_version_patch}")

############TODO Boost
#TODO 18.04的/usr/没有FindBoost.cmake
#TODO 使用Boost_DIR不行，所以这里使用CMAKE_MODULE_PATH
#set(CMAKE_MODULE_PATH "/home/liuhao/software/cmake/cmake-3.16.5-Linux-x86_64/share/cmake-3.16/Modules/" ${CMAKE_MODULE_PATH}) # 设置FindBoost.cmake路径
#set(CMAKE_MODULE_PATH "/home/liuhao/software/clion_and_pycharm/CLion-2020.1/clion-2020.1/bin/cmake/linux/share/cmake-3.16/Modules/" ${CMAKE_MODULE_PATH})
    # Linux 配置 - 使用你的实际路径
    set(BOOST_ROOT "/home/zzz/ros2_ws/src/3rdparty/boost/boost_${boost_version_major}_${boost_version_minor}_${boost_version_patch}")
    set(Boost_NO_SYSTEM_PATHS ON)
    
    # 设置 Boost 组件路径（根据你的实际安装情况调整）
    set(CMAKE_INCLUDE_PATH "${BOOST_ROOT}/install/include" ${CMAKE_INCLUDE_PATH})
    set(CMAKE_LIBRARY_PATH "${BOOST_ROOT}/install/lib" ${CMAKE_LIBRARY_PATH})
    
    # 打印调试信息
    message(STATUS "BOOST_ROOT: ${BOOST_ROOT}")
    message(STATUS "CMAKE_INCLUDE_PATH for Boost: ${CMAKE_INCLUDE_PATH}")
    message(STATUS "CMAKE_LIBRARY_PATH for Boost: ${CMAKE_LIBRARY_PATH}")
endif()

# 查找 Boost - 移除 EXACT 和 MODULE
find_package(Boost ${boost_version} REQUIRED)
include_directories(${CMAKE_INCLUDE_PATH})
link_directories(${CMAKE_LIBRARY_PATH})  # TODO ${Boost_LIBRARY_DIR}不行

list(APPEND LIB_NAMES Boost)

#====================================================================================
#====================================================================================
set(eigen3_version 3.3.7)

############TODO eigen
set(Eigen3_DIR ${${3rdparty_PATH}/eigen/eigen-git-mirror-${eigen3_version}/build_cmake/install/share/eigen3/cmake/})    # 设置Eigen3Config.cmake路径

###eigen3.2.29的话，不要加3.2.29 EXACT
#find_package(Eigen3 ${eigen3_version} EXACT CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH NO_CMAKE_PATH NO_CMAKE_SYSTEM_PATH NO_CMAKE_ENVIRONMENT_PATH)
find_package(Eigen3 ${eigen3_version} EXACT CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)

#include_directories(${EIGEN3_INCLUDE_DIR})    ###TODO 需要加上这行
include_directories(${EIGEN3_INCLUDE_DIRS})    ###TODO 需要加上这行

########TODO pcl1.8.1：Could NOT find eigen (missing: EIGEN_INCLUDE_DIRS)
########TODO pcl1.9.0：Could NOT find Eigen (missing: EIGEN_INCLUDE_DIR)
#####TODO 需要$EIGEN_INCLUDE_DIRS/EIGEN_INCLUDE_DIR，这里设置一下
set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})

#list(APPEND LIB_NAMES Eigen)
#list(APPEND LIB_NAMES EIGEN)
list(APPEND LIB_NAMES Eigen3)
list(APPEND LIB_NAMES EIGEN3)

#====================================================================================
#====================================================================================
###########TODO qt5

set(qt5_version 5.13.2)
set(Qt5_DIR "${3rdparty_PATH}/qt/Qt${qt5_version}/${qt5_version}/msvc2017_64/lib/cmake/Qt5/")
find_package(Qt5 ${qt5_version} EXACT CONFIG REQUIRED COMPONENTS
        Core
        Gui
        Network
        OpenGL
        Qml
        Sql
#        WebKit                ### TODO 5.12.12没有这个
#        WebKitWidgets         ### TODO 5.12.12没有这个
        Widgets
        Xml
        NO_SYSTEM_ENVIRONMENT_PATH
        NO_DEFAULT_PATH
        )
#list(APPEND LIB_NAMES
#        Qt5Core
#        Qt5Gui
#        Qt5Network
#        Qt5OpenGL
#        Qt5Qml
#        Qt5Sql
##        Qt5WebKit
##        Qt5WebKitWidgets
#        Qt5Widgets
#        Qt5Xml
#        )


#set(CMAKE_AUTOMOC ON)    ###负责处理信号槽，生成信号的moc_xxx.cpp文件，main08运行需要开启这个
#set(CMAKE_AUTOUIC ON)    ###负责处理ui文件，生成对应的ui_xxxx.h文件，例如根据mainwindow.ui生成ui_mainwindow.h
#set(CMAKE_AUTORCC ON)    ###负责处理qrc资源文件


#====================================================================================
#====================================================================================
set(vtk_version 9.4.1)


# 使用正则表达式匹配所有数字
string(REGEX MATCHALL "[0-9]+" vtk_version_major_minor ${vtk_version})
#foreach(word IN LISTS vtk_version_major_minor)
#    message("==================${word}")
#endforeach()
list(GET vtk_version_major_minor 0 vtk_version_major)
list(GET vtk_version_major_minor 1 vtk_version_minor)
#message("==================${vtk_version_major}")
#message("==================${vtk_version_minor}")


#############TODO vtk
#############TODO Qt5.9.5有 Qt5WebKitWidgetsConfig.cmake和libQt5WebKitWidgets.so
#############TODO Qt5.12.12没有 Qt5WebKitWidgetsConfig.cmake和libQt5WebKitWidgets.so
#############  ldd ${3rdparty_PATH}/vtk/vtk_6.3.0/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.3.so | grep Qt5

set(VTK_DIR "${3rdparty_PATH}/vtk/VTK-${vtk_version}_${CMAKE_BUILD_TYPE}_qt${qt5_version}/build_cmake/install/lib/cmake/vtk-${vtk_version_major}.${vtk_version_minor}/")
###

find_package(VTK ${vtk_version} EXACT CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)

include_directories(${VTK_INCLUDE_DIRS})    ###TODO 需要加上这行
#link_directories("")
include_directories("${3rdparty_PATH}/vtk/VTK-${vtk_version}_${CMAKE_BUILD_TYPE}_qt${qt5_version}/build_cmake/install/include/vtk-${vtk_version_major}.${vtk_version_minor}/")   # 无法打开包括文件: “vtkSmartPointer.h”: No such file or directory
list(APPEND LIB_NAMES VTK)

set(GL_PATH "C:/Program Files (x86)/Windows Kits//10/Lib/10.0.19041.0/um/x64/")

#====================================================================================
#====================================================================================
#set(lz4_version 1.9.2)
#
#set(LZ4_DIR "${3rdparty_PATH}/lz4/lz4-${lz4_version}_${CMAKE_BUILD_TYPE}/contrib/cmake_unofficial/build_cmake/install/")
#include_directories(${LZ4_DIR}/include/)  ###lz4.h lz4hc.h
#link_directories(${LZ4_DIR}/lib/)


#====================================================================================
#====================================================================================
#set(flann_version 1.9.2)
#
##TODO flann依赖lz4
#set(FLANN_DIR "${3rdparty_PATH}/flann/flann-${flann_version}_${CMAKE_BUILD_TYPE}/build_cmake_lz4_${lz4_version}/install/")
#set(FLANN_ROOT "${3rdparty_PATH}/flann/flann-${flann_version}_${CMAKE_BUILD_TYPE}/build_cmake_lz4_${lz4_version}/install/")
#set(FLANN_INCLUDE_DIR ${FLANN_DIR}/include/)                                 ###TODO 为了配合pcl的FindFLANN.cmake
#set(FLANN_INCLUDE_DIRS ${FLANN_DIR}/include/)                                ###TODO 为了配合pcl的FindFLANN.cmake
##set(FLANN_LIBRARY ${FLANN_DIR}/lib/)                                        ###TODO 为了配合pcl的FindFLANN.cmake
##set(FLANN_LIBRARY ${FLANN_DIR}/lib/flann.lib ${FLANN_DIR}/lib/flann_cpp.lib)###TODO 为了配合pcl的FindFLANN.cmake
#set(FLANN_LIBRARY ${FLANN_DIR}/lib/flann.lib)                                ###TODO 为了配合pcl的FindFLANN.cmake
##set(FLANN_LIBRARIES flann.lib flann_cpp.lib)
#
#include_directories(
#        ${FLANN_DIR}/include/flann/io/
#        ${FLANN_INCLUDE_DIRS}
#)
#link_directories(${FLANN_DIR}/lib/)
##link_libraries(${FLANN_LIBRARIES})
#
#
##set(FLANN_LIBRARY ${FLANN_DIR}/lib/)报错如下：
##错误	LNK1104	无法打开文件“E:\Robot\Project\3rdparty\\flann\flann-1.9.2\build_cmake_lz4_1.9.2\install\\lib\\.obj”
#
##set(FLANN_LIBRARY ${FLANN_DIR}/lib/flann.lib ${FLANN_DIR}/lib/flann_cpp.lib)报错如下：
##错误	LNK1104	无法打开文件“E:\Robot\Project\3rdparty\\flann\flann-1.9.2\build_cmake_lz4_1.9.2\install\\lib\flann.lib;E:\Robot\Project\3rdparty\\flann\flann-1.9.2\build_cmake_lz4_1.9.2\install\\lib\flann_cpp.lib”


#====================================================================================
#====================================================================================
set(pcl_version 1.15.0)


# 使用正则表达式匹配所有数字
string(REGEX MATCHALL "[0-9]+" pcl_version_major_minor ${pcl_version})
#foreach(word IN LISTS pcl_version_major_minor)
#    message("==================${word}")
#endforeach()
list(GET pcl_version_major_minor 0 pcl_version_major)
list(GET pcl_version_major_minor 1 pcl_version_minor)
#message("==================${pcl_version_major}")
#message("==================${pcl_version_minor}")


#########TODO pcl
#########TODO pcl_1.8.1是基于vtk6.3.0编译安装的
#########TODO pcl-pcl-1.8.0是基于vtk6.3.0编译安装的
#########TODO pcl-pcl-1.8.1是基于vtk6.3.0编译安装的
#########TODO pcl-pcl-1.9.0是基于vtk6.3.0编译安装的
#########TODO pcl-pcl-1.9.0_liuhao是基于vtk6.3.0编译安装的
#########TODO pcl1.10.0是基于vtk6.3.0编译安装的
#########  ldd ${3rdparty_PATH}/pcl/pcl_1.8.1/lib/x86_64-linux-gnu/libpcl_visualization.so | grep vtk
#########  ldd ${3rdparty_PATH}/pcl/pcl-pcl-1.8.0暂时不用/build_cmake/install/lib/libpcl_visualization.so | grep vtk
#########  ldd ${3rdparty_PATH}/pcl/pcl-pcl-1.8.1/build_cmake/install/lib/libpcl_visualization.so | grep vtk
#########  ldd ${3rdparty_PATH}/pcl/pcl-pcl-1.9.0暂时不用/build_cmake/install/lib/libpcl_visualization.so | grep vtk
#########  ldd ${3rdparty_PATH}/pcl/pcl-pcl-1.9.0_liuhao暂时不用/build_cmake/install/lib/libpcl_visualization.so | grep vtk
#########  ldd ${3rdparty_PATH}/pcl/pcl-pcl-1.10.0暂时不用/build_cmake/install/lib/libpcl_visualization.so | grep vtk

###
###
###

set(PCL_DIR "${3rdparty_PATH}/pcl/pcl-pcl-${pcl_version}_${CMAKE_BUILD_TYPE}_vtk${vtk_version}/build_cmake/install/cmake/")      # 设置PCLConfig.cmake路径
###
###

find_package(PCL ${pcl_version} EXACT CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)

###去掉WARNING
###去掉WARNING
list(REMOVE_ITEM PCL_LIBRARIES "${3rdparty_PATH}/flann/flann-${flann_version}/build_cmake_lz4_${lz4_version}/install//lib/")
list(REMOVE_ITEM PCL_LIBRARIES "optimized")
###
###

#if(${PCL_VERSION} STREQUAL "1.7.2" OR ${PCL_VERSION} STREQUAL "1.8.1" OR ${PCL_VERSION} STREQUAL "1.10.0")
    include_directories(${PCL_INCLUDE_DIRS})     ###TODO pcl1.7.2 pcl1.8.1需要加上这行
#    link_directories("")    ###TODO libOpenNI.so.0 libOpenNI2.so.0
#    link_directories("")
#endif()


list(APPEND LIB_NAMES PCL)


message("==========Found FLANN_FOUND       : ${FLANN_FOUND}")
message("==========Found FLANN_INCLUDE_DIRS:${FLANN_INCLUDE_DIRS}")
message("==========Found FLANN_LIBRARY_DIRS:${FLANN_LIBRARY_DIRS}")
message("==========Found FLANN_LIBRARIES   :${FLANN_LIBRARIES}")

list(APPEND LIB_NAMES FLANN)


#====================================================================================
#====================================================================================
#set(ffmpeg_version )
set(opencv_version 3.4.7)

#link_directories("")


############TODO opencv
set(OpenCV_DIR "${3rdparty_PATH}/opencv/opencv-${opencv_version}_${CMAKE_BUILD_TYPE}/build_cmake/install/x64/vc16/lib/")        # 设置OpenCVConfig.cmake路径

find_package(OpenCV ${opencv_version} EXACT CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)

list(APPEND LIB_NAMES OpenCV)

#====================================================================================
#====================================================================================
####
#set(cuda_version 11.1)
#
#############TODO cuda
##set(CMAKE_MODULE_PATH "/home/liuhao/software/cmake/cmake-3.16.5-Linux-x86_64/share/cmake-3.16/Modules/" ${CMAKE_MODULE_PATH}) # 设置FindCUDA.cmake路径
#set(CMAKE_MODULE_PATH "${3rdparty_PATH}/cuda/" ${CMAKE_MODULE_PATH}) # 设置FindCUDA.cmake路径
#
##set(CUDA_HOME "/usr/local/cuda-${cuda_version}/")
##set(CUDA_BIN_PATH "/usr/local/cuda-${cuda_version}/")
##set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-${cuda_version}/")
#set(CUDA_TOOLKIT_ROOT_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v${cuda_version}/")
#find_package(CUDA ${cuda_version} EXACT MODULE REQUIRED)
#
#include_directories(${CUDA_INCLUDE_DIRS})
####include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include/)
#include_directories("C:/ProgramData/NVIDIA Corporation/CUDA Samples/v${cuda_version}/common/inc/")	#helper_cuda.h
#
#message("==========Found CUDA_LIBRARIES       :${CUDA_LIBRARIES}")
#message("==========Found CUDA_CUFFT_LIBRARIES :${CUDA_CUFFT_LIBRARIES}")
#message("==========Found CUDA_CUBLAS_LIBRARIES:${CUDA_CUBLAS_LIBRARIES}")
#
#list(APPEND LIB_NAMES CUDA)

#====================================================================================
#====================================================================================

#include_directories("${3rdparty_PATH}/libcbdetect/libcbdetect-master_${CMAKE_BUILD_TYPE}/include/")
#link_directories("${3rdparty_PATH}/libcbdetect/libcbdetect-master_${CMAKE_BUILD_TYPE}/build_cmake/lib/")

#====================================================================================
#====================================================================================

# set(Ubuntu1404_Computer_usr "${3rdparty_PATH}/Ubuntu14.04/Computer/usr/")
#
# set(freenect2_DIR "${3rdparty_PATH}/freenect2/libfreenect2-master/build_cmake/install/lib/cmake/freenect2/")   # 设置freenect2Config.cmake路径
# find_package(freenect2 0.2.0 EXACT CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)
# include_directories(${freenect2_INCLUDE_DIRS})
# list(APPEND LIB_NAMES freenect2)
#
# set(Torch_DIR "${3rdparty_PATH}/libtorch/share/cmake/Torch/")               # 设置TorchConfig.cmake路径
# find_package(Torch 1.1.0 EXACT CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)
# list(APPEND LIB_NAMES Torch)
#
# set(Aubo_DIR "${3rdparty_PATH}/Aubo/share/")                               # 设置AuboConfig.cmake路径
# find_package(Aubo CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)           # 这里没有设置版本号
# include_directories(${Aubo_INCLUDE_DIRS})
# link_directories(${Aubo_LIBRARY_DIRS})
# list(APPEND LIB_NAMES Aubo)
#
# set(Robwork_DIR "${3rdparty_PATH}/robwork-14.04/share/")                      # 设置RobworkConfig.cmake路径
# find_package(Robwork CONFIG REQUIRED NO_SYSTEM_ENVIRONMENT_PATH NO_DEFAULT_PATH)        # 这里没有设置版本号
# include_directories(${Robwork_INCLUDE_DIRS})
# link_directories(${Robwork_LIBRARY_DIRS})
# list(APPEND LIB_NAMES Robwork)

#====================================================================================
#====================================================================================

message("=============================need find libs:${LIB_NAMES}")


FOREACH (LIB_NAME ${LIB_NAMES})
    if (${LIB_NAME}_FOUND)

        message("==========Found ${LIB_NAME}_FOUND=============1====:${${LIB_NAME}_FOUND}")
        message("==========Found ${LIB_NAME}_VERSION===========1====:${${LIB_NAME}_VERSION}")
        message("==========Found ${LIB_NAME}_LIB_VERSION=======1====:${${LIB_NAME}_LIB_VERSION}")

        message("==========Found ${LIB_NAME}_VERSION_MAJOR=====1====:${${LIB_NAME}_VERSION_MAJOR}")
        message("==========Found ${LIB_NAME}_VERSION_MINOR=====1====:${${LIB_NAME}_VERSION_MINOR}")
        message("==========Found ${LIB_NAME}_VERSION_PATCH=====1====:${${LIB_NAME}_VERSION_PATCH}")

        message("==========Found ${LIB_NAME}_MAJOR_VERSION=====1====:${${LIB_NAME}_MAJOR_VERSION}")
        message("==========Found ${LIB_NAME}_MINOR_VERSION=====1====:${${LIB_NAME}_MINOR_VERSION}")
        message("==========Found ${LIB_NAME}_SUBMINOR_VERSION==1====:${${LIB_NAME}_SUBMINOR_VERSION}")

        message("==========Found ${LIB_NAME}_DIR===============2====:${${LIB_NAME}_DIR}")
        message("==========Found ${LIB_NAME}_CONFIG_PATH=======3====:${${LIB_NAME}_CONFIG_PATH}")
        message("==========Found ${LIB_NAME}_INSTALL_PATH======4====:${${LIB_NAME}_INSTALL_PATH}")

        message("==========Found ${LIB_NAME}_INCLUDE_DIR=======5====:${${LIB_NAME}_INCLUDE_DIR}")
        message("==========Found ${LIB_NAME}_INCLUDE_DIRS======5====:${${LIB_NAME}_INCLUDE_DIRS}")
        message("==========Found ${LIB_NAME}_LIBRARY_DIR=======6====:${${LIB_NAME}_LIBRARY_DIR}")
        message("==========Found ${LIB_NAME}_LIBRARY_DIRS======6====:${${LIB_NAME}_LIBRARY_DIRS}")
        message("==========Found ${LIB_NAME}_LIBRARIES=========7====:${${LIB_NAME}_LIBRARIES}")

#        ## 导入头文件的文件夹路径
#        include_directories(${${XXXXX}_INCLUDE_DIRS})     # Find<PackageName>.cmake里面肯定是已经有了这行
#
#        ## 链接库文件的文件夹路径
#        link_directories(${${XXXXX}_LIBRARY_DIRS})        # Find<PackageName>.cmake里面肯定是已经有了这行

    endif ()
    message("")
    message("")
ENDFOREACH ()

#====================================================================================
#====================================================================================

#遍历变量的方式
FOREACH (ele ${CMAKE_MODULE_PATH})
    message("==========Found CMAKE_MODULE_PATH:${ele}")
ENDFOREACH ()


#遍历变量的方式
FOREACH (ele ${CMAKE_PREFIX_PATH})
    message("==========Found CMAKE_PREFIX_PATH:${ele}")
ENDFOREACH ()

#====================================================================================
#====================================================================================

#add_subdirectory(src)

#add_subdirectory(test)

include_directories("./")


# 生成一个共享库，会生成一个libhello_shared.so文件
add_library(PCLRegistration_shared SHARED PCLRegistration.cpp PCLRegistration.h)
#VERSION 指代动态库版本,SOVERSION 指代 API 版本。
#set_target_properties(PCLRegistration_shared PROPERTIES VERSION 1.2 SOVERSION 1)
# 添加导出宏定义（关键步骤）
target_compile_definitions(PCLRegistration_shared PRIVATE MYLIB_EXPORTS)
target_link_libraries(PCLRegistration_shared
        ${Boost_LIBRARIES}
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )


add_executable(main main.cpp)
target_link_libraries(main
        ${Boost_LIBRARIES}
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PROJECT_BINARY_DIR}/lib/Release/PCLRegistration_shared.lib
        )


#====================================================================================
#====================================================================================

