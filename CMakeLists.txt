# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# enforce C++11 features
set(CMAKE_CXX_STANDARD 11)

# produce the cmake var PROJECT_NAME
project(assignment_solve-maze)

# mandatory use of these packages
find_package(ICUBcontrib REQUIRED)
find_package(YARP 3.5.1 REQUIRED COMPONENTS os sig math cv)
find_package(ICUB REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

find_package(OpenCV REQUIRED)

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# maze handler
add_executable(${PROJECT_NAME}-handler ${CMAKE_SOURCE_DIR}/src/handler.cpp)
target_compile_definitions(${PROJECT_NAME}-handler PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}-handler ${YARP_LIBRARIES} ctrlLib ${OpenCV_LIBS})
install(TARGETS ${PROJECT_NAME}-handler DESTINATION bin)

# maze craft
add_executable(${PROJECT_NAME}-craft ${CMAKE_SOURCE_DIR}/src/craft.cpp)
target_compile_definitions(${PROJECT_NAME}-craft PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}-craft ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-craft DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
