cmake_minimum_required(VERSION 3.5)
project(Noise)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

set(SOURCE_FILES Source/main.cpp
			     Include/controlfunction.h
				 Include/examples.h Source/examples.cpp
				 Include/imagecontrolfunction.h Source/imagecontrolfunction.cpp
				 Include/lichtenbergcontrolfunction.h
				 Include/math2d.h Source/math2d.cpp
				 Include/math3d.h Source/math3d.cpp
				 Include/noise.h
				 Include/perlin.h Source/perlin.cpp
				 Include/perlincontrolfunction.h
				 Include/planecontrolfunction.h
				 Include/spline.h Source/spline.cpp
				 Include/utils.h Source/utils.cpp)

add_executable(Noise ${SOURCE_FILES})
target_include_directories(Noise PRIVATE Include)
target_link_libraries(Noise ${OpenCV_LIBS})
