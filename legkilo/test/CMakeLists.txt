cmake_minimum_required(VERSION 3.5)

add_executable(test_tbb test_tbb.cc)

target_link_libraries(test_tbb PRIVATE tbb)