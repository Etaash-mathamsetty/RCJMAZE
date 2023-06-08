cmake_minimum_required(VERSION 3.0)
find_package(PkgConfig)

project(wrapper)

add_executable(wrapper main.cpp)
target_link_libraries(wrapper wiringPi stdc++fs)
target_compile_features(wrapper PUBLIC cxx_std_17)

