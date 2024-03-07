cmake_minimum_required(VERSION 3.10)
project(octomap_tutorial)

include_directories(include)
add_executable(
    voxelgrid
    src/main.cpp
)
