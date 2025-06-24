cmake_minimum_required(VERSION 3.0)

project(saya)

add_executable(saya main.cpp)

install(TARGETS saya RUNTIME DESTINATION bin)
