file(GLOB_RECURSE VMAT_TEST_SOURCES
  *.cc
  *.cpp
)

message(STATUS ${VMAT_TEST_SOURCES})
add_executable(vmat_test_all ${VMAT_TEST_SOURCES})

target_include_directories(vmat_test_all PUBLIC "../include")
enable_testing()
find_package(GTest CONFIG REQUIRED)
target_link_libraries(vmat_test_all PRIVATE GTest::gtest_main GTest::gtest GTest::gmock GTest::gmock_main)
# if (NOT CMAKE_CXX_COMPILER MATCHES MSVC)
  # target_link_libraries(vmutils_test_all pthread)
# endif()

include(GoogleTest)
gtest_add_tests(vmat_test_all "" AUTO)
install(TARGETS vmat_test_all LIBRARY DESTINATION "lib" RUNTIME DESTINATION "bin" ARCHIVE DESTINATION "lib")
