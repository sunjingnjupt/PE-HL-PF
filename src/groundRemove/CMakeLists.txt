# cmake_minimum_required(VERSION 2.8.3)
include_directories(include)
include_directories(qt)

add_library(cloud SHARED src/cloud.cpp)
add_library(bin SHARED src/bin.cpp)
add_library(segment SHARED src/segment.cpp)
# add_library(utils SHARED src/utils.cpp) # has been defined in qt sub_derictories



set(GROUND_REMOVE_SET
    bin
    segment
    # component_clustering
    utils
    # cloud
    )
find_package(Threads REQUIRED)
add_library(groundRemove SHARED src/groundRemove.cpp)
target_link_libraries(groundRemove ${GROUND_REMOVE_SET} cloud ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(component_clustering src/component_clustering.cpp)
target_link_libraries(component_clustering ${OpenCV_LIBS} cloud ${Boost_LIBRARIES})

add_library(convex_hull src/convex_hull.cpp)
target_link_libraries(convex_hull ${OpenCV_LIBS} cloud ${Boost_LIBRARIES})

# add_executable(Text3 src/convex_hull.cpp)
# target_link_libraries(Text3 ${OpenCV_LIBS} cloud ${Boost_LIBRARIES})

# add_executable(Test src/test.cpp)

add_library(box_fitting src/box_fitting.cpp)
target_link_libraries(box_fitting ${OpenCV_LIBS} cloud  convex_hull ${Boost_LIBRARIES})

# find_package(Eigen3 REQUIRED Core)
# include_directories(${EIGEN3_INCLUDE_DIR})

# add_library(imm_ukf_pda src/imm_ukf_pda.cpp src/ukf.cpp)
# target_link_libraries(imm_ukf_pda ${OpenCV_LIBS} cloud ${Boost_LIBRARIES})

add_executable(Test src/test2.cpp)
target_link_libraries(Test 
                    ${OpenCV_LIBS}
                    cloud
                    ${Boost_LIBRARIES}
                    ${QGLVIEWER_LIBRARY}
                    ${MY_QT_LIBRARIES}
                    ${OPENGL_gl_LIBRARY}
                    ${OPENGL_glu_LIBRARY})





