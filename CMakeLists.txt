cmake_minimum_required(VERSION 3.9)
project(vmat)

add_library(VMat INTERFACE)
target_include_directories(VMat
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(VMat INTERFACE cxx_std_14)

option(VMAT_INSTALL "install VMat headers" ON)
if (VMAT_INSTALL)
  install(
    DIRECTORY include/VMat
    DESTINATION include
  )
endif()