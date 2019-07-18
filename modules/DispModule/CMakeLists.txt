# Copyright: 2019 IIT
# Author: Malafronte Damiano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(DispModule)

#add_subdirectory(imgui/examples)
#add_subdirectory(imgui/examples/libs/gl3w/GL)
add_subdirectory(sgbm_cuda)

set(source DispModule.cpp StereoMatcher.cpp cvgui.cpp ${IMGUI_SOURCES})
set(header DispModule.h StereoMatcher.h StereoMatcher_enums.h  cvgui.h)

#find_package(SDL2 REQUIRED)
#FIND_PACKAGE(OpenGL REQUIRED)
#find_package(GLEW 3.0 REQUIRED)

find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

#find_package(SGBM_CUDA REQUIRED)

#include_directories(${SDL2_INCLUDE_DIRS})
#include_directories(${GLEW_INCLUDE_DIRS})
#include_directories(${GL3W_INCLUDE_DIRS})
#include_directories(${OPENGL_INCLUDE_DIRS})
#include_directories("imgui/examples/libs/gl3w")

include_directories(${stereoVision_INCLUDE_DIRS})
#include_directories(imgui)
#include_directories(imgui/examples)
include_directories("/home/damiano/robotology/forks/stereo-vision/modules/DispModule/sgbm_cuda")


if(USE_SIFT_GPU)
    include_directories(${SIFTGPU_INCLUDE_DIRS})
endif()

#SET(GCC_COVERAGE_LINK_FLAGS    "-lGL -ldl")
#add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
#SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

add_executable(${PROJECT_NAME} ${source} ${header})

if(USE_SIFT_GPU)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USING_GPU)
endif()

include_directories("/home/damiano/robotology/forks/stereo-vision/modules/DispModule/sgbm_cuda")
#find_package(SGBM_CUDA REQUIRED)

#target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${SDL2_LIBRARIES} GUI_DISP GL_MINE SGBM_CUDA ${OPENGL_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${SDL2_LIBRARIES} SGBM_CUDA ${OPENGL_LIBRARIES} ${CMAKE_DL_LIBS})

if(USE_SIFT_GPU)
	target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
