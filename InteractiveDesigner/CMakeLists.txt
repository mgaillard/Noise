add_executable(InteractiveDesigner)

message(STATUS "Creating target 'InteractiveDesigner'")

set(UI_FILES
    source/mainwindow.ui
    source/parameterdock.ui
)

set(HEADER_FILES
    include/displaywidget.h
    include/mainwindow.h
    include/noiseparameters.h
    include/noiserenderer.h
    include/parameterdock.h
)

set(SRC_FILES
    source/displaywidget.cpp
    source/mainwindow.cpp
    source/main.cpp
    source/noiserenderer.cpp
    source/parameterdock.cpp
)

# Setup filters in Visual Studio
source_group("Forms" FILES ${UI_FILES})
source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

target_sources(InteractiveDesigner
    PUBLIC
    ${HEADER_FILES}
    PRIVATE
    ${SRC_FILES}
    ${UI_FILES}
)

target_include_directories(InteractiveDesigner
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(InteractiveDesigner 
    PRIVATE
    NoiseLib
    Qt6::Core
    Qt6::Widgets
    Qt6::Gui
    Qt6::Concurrent
)

if(TARGET Qt6::windeployqt)
    add_custom_command(TARGET InteractiveDesigner
        POST_BUILD
        COMMAND set PATH=%PATH%$<SEMICOLON>${qt6_install_prefix}/bin
        COMMAND Qt6::windeployqt "$<TARGET_FILE_DIR:InteractiveDesigner>/$<TARGET_FILE_NAME:InteractiveDesigner>"
    )
endif()
