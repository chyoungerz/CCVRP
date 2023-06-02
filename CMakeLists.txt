cmake_minimum_required(VERSION 3.0.0)
project(CCVRP VERSION 0.1.0)

# 为当前路径以及子目录的源文件加入由-D预编译定义
add_definitions(-DWINDOWS)

# 设置C++编译参数(CMAKE_CXX_FLAGS是全局变量)
set(CMAKE_CXX_FLAGS "-Wall -std=gnu++23")

# 设置指定的C++编译器版本是必须的，如果不设置，或者为OFF，则指定版本不可用时，会使用上一版本。
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指定为C++11 版本
set(CMAKE_CXX_STANDARD 23)

#编译选项
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g")
set(CMAKE_CXX_FLAGS_Release "${CMAKE_C_FLAGS_Release} -O3")

#头文件目录
#include_directories()

# 添加子目录
#add_subdirectory()

#源文件目录
#aux_source_directory(. DIR_SRCS)

# 分别设置Debug和Release输出目录
set(CMAKE_LIBRATY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug/lib)
set(CMAKE_LIBRATY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release/bin)

# 指定生成目标 
#add_executable(vrp ${DIR_SRCS})
add_executable(vrp main.cpp node.cpp utils.cpp operator.cpp solution.cpp NSearch.cpp distribution.cpp heuristic.cpp)