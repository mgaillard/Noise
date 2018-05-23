cmake_minimum_required(VERSION 3.5)
project(Worley)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

target_include_directories(Worley PRIVATE "Include")

set(SOURCE_FILES main.cpp Include/math2d.cpp Source/math2d.cpp Include/noise.cpp Source/noise.cpp Inckude/perlin.cpp Source/perlin.cpp Include/worley.cpp Source/worley.cpp)
add_executable(Worley ${SOURCE_FILES})
target_link_libraries(Worley ${OpenCV_LIBS})
