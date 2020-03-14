cmake_minimum_required(VERSION 2.0)
set(CMAKE_CXX_STANDARD 14)
project(vmat)

add_library(VMat INTERFACE)
target_include_directories(VMat INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
