cmake_minimum_required(VERSION 3.27.0)
project(CCVRP VERSION 0.1.0)

# 为当前路径以及子目录的源文件加入由-D预编译定义
add_definitions(-DWINDOWS)

# 设置C++编译参数(CMAKE_CXX_FLAGS是全局变量)
set(CMAKE_CXX_FLAGS "-Wall -std=gnu++23")

# 设置指定的C++编译器版本是必须的，如果不设置，或者为OFF，则指定版本不可用时，会使用上一版本。
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指定为C++23 版本
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
add_executable(vrp main.cpp operator.cpp solution.cpp NSearch.cpp heuristic.cpp)

# 测试
enable_testing()
add_test(NAME p01 COMMAND vrp p01 data01.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p02 COMMAND vrp p02 data02.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p03 COMMAND vrp p03 data03.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p04 COMMAND vrp p04 data04.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p05 COMMAND vrp p05 data05.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p06 COMMAND vrp p06 data06.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p07 COMMAND vrp p07 data07.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p08 COMMAND vrp p08 data08.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p09 COMMAND vrp p09 data09.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)
add_test(NAME p10 COMMAND vrp p10 data10.txt CONFIGURATIONS Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/Testing/p)

#set_tests_properties(p01 PROPERTIES PASS_CONFIGURE "0")