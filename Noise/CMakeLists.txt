add_executable(Noise)

message(STATUS "Creating target 'Noise'")

set(HEADER_FILES
    Include/controlfunction.h
    Include/examples.h
    Include/imagecontrolfunction.h
    Include/lichtenbergcontrolfunction.h
    Include/math2d.h
    Include/math3d.h
    Include/noise.h
    Include/perlin.h
    Include/perlincontrolfunction.h
    Include/planecontrolfunction.h
    Include/spline.h
    Include/utils.h
)

set(SRC_FILES
    Source/main.cpp
    Source/examples.cpp
    Source/imagecontrolfunction.cpp
    Source/math2d.cpp
    Source/math3d.cpp
    Source/perlin.cpp
    Source/spline.cpp
    Source/utils.cpp
)

# Setup filters in Visual Studio
source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

# Copy the ReSharper config file
configure_file(
    "${CMAKE_SOURCE_DIR}/Noise.sln.DotSettings"
    ${CMAKE_BINARY_DIR} COPYONLY
)

target_sources(Noise
    PUBLIC
    ${HEADER_FILES}
    PRIVATE
    ${SRC_FILES}
)

target_include_directories(Noise
    PRIVATE
    Include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(Noise 
    PRIVATE
    OpenMP::OpenMP_CXX
    ${OpenCV_LIBS}
)
