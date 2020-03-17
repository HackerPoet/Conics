cmake_minimum_required(VERSION 3.0.0)

# prject name
project(Conics)

# compiler all *.cpp files in the source directory
file(GLOB SOURCES "*.cpp")
set(BIN_NAME conics)
add_executable(${BIN_NAME} ${SOURCES})
# requirec c++11 standard
target_compile_features(${BIN_NAME} PUBLIC cxx_std_11)

# find dependencies
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")
set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL REQUIRED)
find_package(SFML 2 REQUIRED COMPONENTS graphics window system)
find_package(Eigen3 REQUIRED)

# link dependencies
target_link_libraries(${BIN_NAME} ${OPENGL_LIBRARIES})
include_directories(${SFML_INCLUDE_DIR})
target_link_libraries(${BIN_NAME} ${SFML_LIBRARIES})
include_directories(${EIGEN3_INCLUDE_DIR})


# "make run" target
add_custom_target(run
    COMMAND ${BIN_NAME}
    DEPENDS ${BIN_NAME}
	WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
