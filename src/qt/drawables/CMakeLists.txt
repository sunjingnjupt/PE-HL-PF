include_directories(drawables)
add_library(drawable SHARED
            drawable_cloud.cpp
            drawable_line.cpp
            drawable_selectable_cloud.cpp
            )

target_link_libraries(drawable
                     utils
                    ${QGLVIEWER_LIBRARY}
                    ${MY_QT_LIBRARIES}
                    ${OPENGL_gl_LIBRARY}
                    ${OPENGL_glu_LIBRARY})
                    
                    