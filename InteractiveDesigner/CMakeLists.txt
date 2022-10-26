add_executable(InteractiveDesigner)

message(STATUS "Creating target 'InteractiveDesigner'")

# Files for the project
set(RESOURCE_FILES mainwindow.qrc)

set(UI_FILES
    mainwindow.ui
    parameterdock.ui
)

set(HEADER_FILES
    displaywidget.h
    mainwindow.h
    noiseparameters.h
    noiserenderer.h
    parameterdock.h
)

set(SRC_FILES
    displaywidget.cpp
    mainwindow.cpp
    main.cpp
    noiserenderer.cpp
    parameterdock.cpp
)

# Setup filters in Visual Studio
source_group("Resources" FILES ${RESOURCE_FILES})
source_group("Forms" FILES ${UI_FILES})
source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

target_sources(InteractiveDesigner
    PUBLIC
    ${HEADER_FILES}
    PRIVATE
    ${SRC_FILES}
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
