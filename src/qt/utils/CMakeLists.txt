add_library(utils SHARED utils.cpp)
target_link_libraries(utils ${Boost_LIBRARIES})

add_library(object SHARED object.cpp)
