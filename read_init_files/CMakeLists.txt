cmake_minimum_required(VERSION 3.00)

set(CMAKE_CXX_STANDARD 17)


project(inipp)


add_executable(example example.cpp inipp.h)
