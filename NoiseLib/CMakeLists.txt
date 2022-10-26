add_library(NoiseLib STATIC)

message(STATUS "Creating target 'NoiseLib'")

set(HEADER_FILES
    include/controlfunction.h
    include/imagecontrolfunction.h
    include/lichtenbergcontrolfunction.h
    include/math2d.h
    include/math3d.h
    include/noise.h
    include/perlin.h
    include/perlincontrolfunction.h
    include/planecontrolfunction.h
    include/spline.h
    include/utils.h
)

set(SRC_FILES
    source/imagecontrolfunction.cpp
    source/math2d.cpp
    source/math3d.cpp
    source/perlin.cpp
    source/spline.cpp
    source/utils.cpp
)

# Setup filters in Visual Studio
source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

target_sources(NoiseLib
    PUBLIC
    ${HEADER_FILES}
    PRIVATE
    ${SRC_FILES}
)

target_include_directories(NoiseLib
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(NoiseLib 
    PUBLIC
    OpenMP::OpenMP_CXX
    ${OpenCV_LIBS}
)
