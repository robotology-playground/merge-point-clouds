CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(PROJECTNAME merge-point-clouds)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})
							  
include(YarpInstallationHelpers) 	
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES})

add_subdirectory(app)

add_subdirectory(exploreTorso)
add_subdirectory(manager)
add_subdirectory(mergeClouds)
icubcontrib_add_uninstall_target()

