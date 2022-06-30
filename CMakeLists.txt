cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(voxel_carving)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
# Configure Source & Binary Directories ---------------------------------------
set(PROJECT_ROOT "${PROJECT_SOURCE_DIR}")
set(PROJECT_SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")
# set (OpenCV_DIR /home/cmake/opencv/compiled CACHE PATH "Path to installed OpenCV")
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

set(HEADER_FILES 
    Eigen.h
    VoxelGrid.h
)

set(SOURCE_FILES 
    VoxelGrid.cpp
)

link_directories(${FreeImage_LIBRARY_DIR})
add_executable(voxel_carving main.cpp)
target_link_libraries(voxel_carving ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(voxel_carving PUBLIC ${EIGEN3_INCLUDE_DIR})
target_sources(voxel_carving PRIVATE src/ImageLoader.cpp)

# Unit Tests
add_executable(test_voxel_carving test/Test_ImageLoader.cpp)
target_link_libraries(test_voxel_carving ${OpenCV_LIBS} Eigen3::Eigen gtest gtest_main)
target_include_directories(voxel_carving PUBLIC ${EIGEN3_INCLUDE_DIR})
add_test( test_voxel_carving test_voxel_carving )
target_sources(test_voxel_carving PRIVATE src/ImageLoader.cpp)

# For mac os use additional compile flag
if(MACOS)
    target_compile_options(voxel_carving PRIVATE -mmacosx-version-min=12.0)
    target_compile_options(test_voxel_carving PRIVATE -mmacosx-version-min=12.0)
endif(MACOS)

# On Windows copy dll to executable folder
if(WIN32)
    # Visual Studio properties
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT voxel_carving)
    set_property(TARGET voxel_carving PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
endif(WIN32)
