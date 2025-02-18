cmake_minimum_required(VERSION 3.5)
project(ros2_odrive)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE sources src/*.cpp)

add_executable(main ${sources})
target_include_directories(main PUBLIC include)

set_target_properties(main
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
