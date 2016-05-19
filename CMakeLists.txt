project(rcservice)
cmake_minimum_required(VERSION 2.8)

# local machine header files location
if(RASPI_ROOT_DIR)
    message(STATUS "Raspberry root directory set.")
    include(CMakeForceCompiler)
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR arm)
    set(CMAKE_SYSTEM_VERSION 1)
    set(CMAKE_CXX_COMPILER armv6j-hardfloat-linux-gnueabi-g++)
    set(CMAKE_C_COMPILER armv6j-hardfloat-linux-gnueabi-gcc)
else()
    message(FATAL_ERROR "Raspberry root directory not set.")
endif()

# require c++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()

set(CMAKE_FIND_ROOT_PATH "${RASPI_ROOT_DIR}")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
#set(CMAKE_SYSROOT "${RASPI_ROOT_DIR}")

include_directories("${RASPI_ROOT_DIR}/opt/vc/include")
include_directories("${RASPI_ROOT_DIR}/opt/vc/include/vcos")
include_directories("${RASPI_ROOT_DIR}/opt/vc/include/vcos/pthreads")
include_directories("${RASPI_ROOT_DIR}/opt/vc/include/vmcs_host/linux")
link_directories("${RASPI_ROOT_DIR}/usr/lib/arm-linux-gnueabihf")
link_directories("${RASPI_ROOT_DIR}/lib/arm-linux-gnueabihf")
link_directories("${RASPI_ROOT_DIR}/usr/lib")

# require opencv
find_package(OpenCV REQUIRED core imgproc video highgui)
message(STATUS "OpenCV: ${OpenCV_DIR}")
message(STATUS "OpenCV includes: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OPENCV_INCLUDE_DIRS})
list(APPEND PROJECT_LIBS ${OpenCV_LIBS})

# require threads
#find_package(Threads REQUIRED)
#list(APPEND PROJECT_LIBS ${CMAKE_THREAD_LIBS_INIT})

# header files
include_directories(src)
include_directories(3rdparty)

# serial library
include_directories(3rdparty/serial/include)
file(GLOB_RECURSE SRC_LIST 3rdparty/serial/src/*.cc)

# msppg protocol
aux_source_directory(3rdparty/msppg SRC_LIST)

# require raspicam
#find_package(raspicam REQUIRED)
#list(APPEND PROJECT_LIBS ${raspicam_LIBS})
#aux_source_directory(3rdparty/raspicam SRC_LIST)

# require raspicam
#include("cmake/Findraspicam.cmake")
include_directories("${RASPI_ROOT_DIR}/usr/local/include")
link_directories("${RASPI_ROOT_DIR}/opt/vc/lib")
link_directories("${RASPI_ROOT_DIR}/usr/local/lib")
set(raspicam_LIBS libmmal_core.so;libmmal_util.so;libmmal.so)
list(APPEND PROJECT_LIBS ${raspicam_LIBS})
list(APPEND PROJECT_LIBS "raspicam;raspicam_cv")

# main executable
aux_source_directory(src SRC_LIST)
aux_source_directory(src/uavobjects SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBS})

# test executable
#aux_source_directory(test TEST_LIST)
#add_executable(${PROJECT_NAME}_test ${SRC_LIST} ${TEST_LIST})
#target_link_libraries(${PROJECT_NAME}_test ${PROJECT_LIBS})
#set_target_properties(${PROJECT_NAME}_test PROPERTIES COMPILE_DEFINITIONS "_NO_MAIN")

message(STATUS "Libs: ${PROJECT_LIBS}")
