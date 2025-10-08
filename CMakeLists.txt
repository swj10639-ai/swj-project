cmake_minimum_required(VERSION 3.13)
project(SAT_TR  CXX)
set(CMAKE_CXX_STANDARD 17)

# 设置Z3库的路径
set(Z3_INCLUDE_DIRS /usr/include)  # Z3头文件路径
set(Z3_LIBRARIES /usr/lib/libz3.so)  # Z3库文件路径
include_directories(.)

add_executable(SAT_TR
        edge.cpp
        edge.h
        graph.cpp
        graph.h
        instance.cpp
        instance.h
        mapf_main.cpp
        mapf_main.h
        train.h
        train.cpp
)

# 添加Z3库的头文件路径
target_include_directories(SAT_TR PRIVATE ${Z3_INCLUDE_DIRS})
# 链接 Z3 库
target_link_libraries(SAT_TR  ${Z3_LIBRARIES})

