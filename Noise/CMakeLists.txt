add_executable(Noise)

message(STATUS "Creating target 'Noise'")

set(HEADER_FILES
    examples.h
)

set(SRC_FILES
    main.cpp
    examples.cpp
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

target_link_libraries(Noise 
    PRIVATE
    NoiseLib
)
