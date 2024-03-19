cmake_minimum_required(VERSION 3.22)
project(Nomospline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

list(APPEND CMAKE_PREFIX_PATH "~/Qt/6.6.1/gcc_64")

find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        OpenGL
        OpenGLWidgets
        Qml
        Quick
        REQUIRED)

include_directories(include)

SET(ENGINE_SRC src/engine/MeshLoader.cpp src/engine/Camera.cpp src/engine/SplineLoader.cpp)
SET(WIDGET_SRC
        include/widget/scene.h
        src/widget/scene.cpp
        include/widget/WindowUtils.h
        src/widget/WindowUtils.cpp

)

SET(STRUCTURES_SRC
        src/structures/SplineUtils.cpp)

set(RENDER_SRC
        include/render_object/RenderObject.h
        src/render_object/RenderObject.cpp
        include/render_object/MeshRenderer.h
        src/render_object/MeshRenderer.cpp
        include/render_object/BackgroundRenderer.h
        src/render_object/BackgroundRenderer.cpp
        include/engine/SceneData.h
        src/engine/SceneData.cpp
        include/render_object/GizmosRenderer.h
        src/render_object/GizmosRenderer.cpp
        include/render_object/SplineRenderer.h
        src/render_object/SplineRenderer.cpp
        include/render_object/PointRenderer.h
        src/render_object/PointRenderer.cpp
)
set(OTHER_SRC
        src/timer.cpp
)


add_executable(Nomospline main.cpp ${ENGINE_SRC} ${WIDGET_SRC} ${STRUCTURES_SRC} ${OTHER_SRC} ${RENDER_SRC} shaders.qrc images.qrc)

target_link_libraries(Nomospline
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::OpenGL
        Qt::OpenGLWidgets
        Qt::Qml
        Qt::Quick
        )



if (WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(DEBUG_SUFFIX)
    if (MSVC AND CMAKE_BUILD_TYPE MATCHES "Debug")
        set(DEBUG_SUFFIX "d")
    endif ()
    set(QT_INSTALL_PATH "${CMAKE_PREFIX_PATH}")
    if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
        set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
            set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        endif ()
    endif ()
    if (EXISTS "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/platforms/")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/platforms/")
    endif ()
    foreach (QT_LIB Core Gui Widgets OpenGL OpenGLWidgets)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${QT_INSTALL_PATH}/bin/Qt6${QT_LIB}${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>")
    endforeach (QT_LIB)
endif ()
