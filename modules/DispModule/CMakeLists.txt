# Copyright: 2019 IIT
# Author: Malafronte Damiano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(DispModule)

add_subdirectory(sgbm_cuda)

set(USE_DISPMODULE_GUI ON CACHE BOOL "Select ON to compile with the GUI code")

set(source DispModule.cpp StereoMatcher.cpp)
set(header DispModule.h StereoMatcher.h StereoMatcher_enums.h)

if(USE_DISPMODULE_GUI)
    list(APPEND source cvgui.cpp)
    list(APPEND header cvgui.h)
endif()

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})

include_directories(${stereoVision_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/sgbm_cuda")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DALLOW_IDL_GENERATION=ON")

#set(${PROJECT_NAME}_THRIFT_HDR thrift/DispModule_IDL.thrift)
#yarp_add_idl(${PROJECT_NAME}_THRIFT_SRC ${${PROJECT_NAME}_THRIFT_HDR})

add_executable(${PROJECT_NAME} ${source} ${header})

if(USE_SIFT_GPU)
    include_directories(${SIFTGPU_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE USING_GPU)
endif()

if(USE_DISPMODULE_GUI)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_GUI)
endif()

target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} SGBM_CUDA ${CMAKE_DL_LIBS})

if(USE_SIFT_GPU)
    target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

#set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")

#yarp_idl_to_dir(INPUT_FILES DispModule_IDL.thrift
#                OUTPUT_DIR ${generated_libs_dir}
#                SOURCES_VAR sources
#                HEADERS_VAR headers
#                INCLUDE_DIRS_VAR include_dirs)




