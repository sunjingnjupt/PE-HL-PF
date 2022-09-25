cmake_minimum_required(VERSION 2.8.3)
project(lidarCode)

add_definitions(-Wall)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# if has no defined build set it to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${PROJECT_SOURCE_DIR}/cmake_modules"
    )

# Test Tools
include(CTest)
IF(IS_DIRECTORY "/usr/src/gtest/")
    MESSAGE(STATUS "Found google test sources in /usr/src/gtest/")

    ADD_SUBDIRECTORY(/usr/src/gtest/ gtest)

    # mimick the behaviour of find_package(GTest)
    SET(GTEST_FOUND TRUE)
    SET(GTEST_BOTH_LIBRARIES gtest gtest_main)
    SET(GTEST_LIBRARIES gtest)
    SET(GTEST_MAIN_LIBRARIES gtest_main)
ELSE()
    find_package(GTest)
ENDIF()

set(Boost_USE_STATIC_LIBS OFF)

# Qt5
set(Qt5_DIR "/opt/Qt5.6.0/5.6/gcc_64/lib/cmake/Qt5")
set(Qt5_Widgets "/opt/Qt5.6.0/5.6/gcc_64/lib/cmake/Qt5Widgets")
find_package(Qt5 COMPONENTS Core Widgets Gui Xml OpenGL REQUIRED)
include_directories(${Qt5Core_INCLUDE_DIRS}
                    ${Qt5Xml_INCLUDE_DIRS}
                    ${Qt5Gui_INCLUDE_DIRS}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5OpenGL_INCLUDE_DIRS})
set(MY_QT_LIBRARIES
                    ${Qt5Widgets_LIBRARIES}
                    ${Qt5Core_LIBRARIES}
                    ${Qt5Gui_LIBRARIES}
                    ${Qt5Xml_LIBRARIES}
                    ${Qt5OpenGL_LIBRARIES}
                    ${Boost_LIBRARIES}
		    /usr/lib/x86_64-linux-gnu/libtiff.so.5
                    )

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
# for opencv
set (OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV 3.4 REQUIRED
            PATHS OpenCV_DIR
            NO_DEFAULT_PATH)

# for opengl
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE})

#for QGLViewer
find_package(QGLViewer REQUIRED)
include_directories(
    ${PROJECT_BINARY_DIR}/src
    ${QGLVIEWER_INCLUDE_DIR}
    )

# for Eigen3
find_package(Eigen3 REQUIRED Core)
include_directories(${EIGEN3_INCLUDE_DIR})

# for Boost
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
include_directories(${Boost_INCLUDE_DIRS})       
# find_package(PCL QUIET COMPONENTS common io)

find_package(QGLViewer REQUIRED)
message(STATUS "Linking against Qt libs: ${MY_QT_LIBRARIES}")
message(STATUS "Linking against QGlViewer lib: ${QGLVIEWER_LIBRARY}")

include_directories(
                    src
                    ${PROJECT_BINARY_DIR}/src   
)

add_subdirectory(src)

# cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
# make


