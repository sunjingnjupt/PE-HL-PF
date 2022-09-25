cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

include_directories(groundRemove/include)
include_directories(
    utils
    drawables
    viewer
    widgets   
)

add_subdirectory(utils)
add_subdirectory(drawables)
add_subdirectory(viewer)
add_subdirectory(widgets)

add_executable(QTest main.cpp)
target_link_libraries(QTest qt_widgets groundRemove component_clustering box_fitting)

