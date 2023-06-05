cmake_minimum_required(VERSION 3.9)
project(vmat)
set(CMAKE_CXX_STANDARD 17)

add_library(VMat INTERFACE)
target_include_directories(VMat
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

option(BUILD_VMAT_TEST "Set ON to build vmat test" OFF)
if(BUILD_VMAT_TEST)
add_subdirectory(test)
endif()

option(VMAT_INSTALL "install VMat headers" ON)
if (VMAT_INSTALL)
  install(
    DIRECTORY include/VMat
    DESTINATION include
  )
endif()
