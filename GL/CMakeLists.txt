cmake_minimum_required(VERSION 3.18)
project(OpenGLProject)

set(CMAKE_CXX_STANDARD 11)

set(GLUT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(GLUT_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/GL")
#find_package(GLUT REQUIRED)

message(${GLUT_INCLUDE_DIR})

include_directories(${GLUT_INCLUDE_DIR})
link_directories(${GLUT_LIB_DIR})

add_executable(OpenGLProject main.cpp)

#target_link_libraries(OpenGLProject glut32 -lopengl32 -lglu32)
target_link_libraries(OpenGLProject "${GLUT_LIB_DIR}/glut.lib" -lopengl32 "${GLUT_LIB_DIR}/glut32.lib")
