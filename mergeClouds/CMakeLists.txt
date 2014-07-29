CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME merge_point_clouds)
PROJECT(${PROJECTNAME})

find_package(YARP)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${YARP_INCLUDE_DIRS})
add_executable(merge_point_clouds merge_point_clouds.cpp)
target_link_libraries(merge_point_clouds ${YARP_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS merge_point_clouds  RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../bin)







