# Copyright: 2019 IIT
# Author: Malafronte Damiano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(DispModule)

add_subdirectory(imgui/examples)
add_subdirectory(imgui/examples/libs/gl3w/GL)


set(source DispModule.cpp gui.cpp ${IMGUI_SOURCES})
set(header DispModule.h gui.h)

find_package(SDL2 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
find_package(GLEW 3.0 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GL3W_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories("imgui/examples/libs/gl3w")

include_directories(${stereoVision_INCLUDE_DIRS})
include_directories(imgui)
include_directories(imgui/examples)

if(USE_SIFT_GPU)
    include_directories(${SIFTGPU_INCLUDE_DIRS})
endif()

SET(GCC_COVERAGE_LINK_FLAGS    "-lGL -ldl")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

add_executable(${PROJECT_NAME} ${source} ${header})

if(USE_SIFT_GPU)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USING_GPU)
endif()

target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${SDL2_LIBRARIES} GUI_DISP GL_MINE ${OPENGL_LIBRARIES} ${CMAKE_DL_LIBS})

if(USE_SIFT_GPU)
	target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
