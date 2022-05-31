# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(my-project)

# mandatory use of these packages
find_package(YARP 3.5.1 REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(Gazebo REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

add_subdirectory(gazebo)

# assignment
include_directories(${CMAKE_SOURCE_DIR}/src)
# Controller to move to the box
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp
                               )
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
# Force controller
add_executable(force ${CMAKE_SOURCE_DIR}/src/force.cpp
                               )
target_compile_definitions(force PRIVATE _USE_MATH_DEFINES)
target_link_libraries(force ${YARP_LIBRARIES})
install(TARGETS force DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
