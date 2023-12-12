cmake_minimum_required(VERSION 3.2)
project(LoFTR_SLAM_from_scratch)

set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_OSX_ARCHITECTURES "arm64")
set(CMAKE_CXX_STANDARD 14)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Boost
find_package( Boost COMPONENTS program_options REQUIRED )
#find_package(Boost REQUIRED COMPONENTS program_options)
include_directories( ${Boost_INCLUDE_DIR} )

# Eigen
find_package( Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR})

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# G2O
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIR} )

# PCL
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(${PCL_INCLUDE_DIRS})
include_directories(/usr/include/vtk-9.1)

# Panolin
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )

# OnnxRuntime
find_path(ONNX_RUNTIME_SESSION_INCLUDE_DIRS onnxruntime_cxx_api.h HINTS /home/hxl/Thirdparty/onnxruntime-linux-x64-gpu-1.9.0/include)
include_directories(${ONNX_RUNTIME_SESSION_INCLUDE_DIRS})

find_library(ONNX_RUNTIME_LIB onnxruntime HINTS /home/hxl/Thirdparty/onnxruntime-linux-x64-gpu-1.9.0/lib)

set( THIRD_PARTY_LIBS
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_TYPES_SBA}
        ${G2O_TYPES_SIM3}
        ${G2O_SOLVER_DENSE}
        ${G2O_SOLVER_EIGEN}
        ${G2O_TYPES_SLAM3D}
        ${Pangolin_LIBRARIES}
        ${ONNX_RUNTIME_LIB}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        /usr/lib/x86_64-linux-gnu/libboost_serialization.so
        fmt
        ssl
        crypto
        )

############### dependencies ######################
include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/CameraModels )
add_subdirectory(src)
add_subdirectory( test )

