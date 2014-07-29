CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(PROJECTNAME merge-clouds-manager)
project(${PROJECTNAME})
							  
set(sources main.cpp)
source_group("Source Files" FILES ${sources})

include_directories(${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${sources})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

