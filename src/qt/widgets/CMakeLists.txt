set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(SKIP_AUTOMOC ON)
set(SKIP_AUTOUIC ON)

set(QT_SOURCES
    base_viewer_widget.cpp
    mainwindow.cpp
    )

set(QT_HEADERS
    base_viewer_widget.h
    mainwindow.h
    )

set(QT_UI_FILES
    ui/mainwindow.ui
    )

if(Qt5_FOUND)
    QT5_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
    QT5_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})
else()
    message(FATAL_ERROR "Qt not found. So cannot moc.")
endif()

add_library(qt_widgets
            SHARED
            ${QT_SOURCES}
            ${QT_UI_FILES_MOC}
            ${QT_HEADERS_MOC}
            )

target_link_libraries(qt_widgets
                    viewer
                    drawable
                    utils
                    ${OpenCV_LIBS}
                    ${QGLVIEWER_LIBRARY}
                    ${MY_QT_LIBRARIES}
                    ${OPENGL_gl_LIBRARY}
                    ${OPENGL_glu_LIBRARY}
                    )
