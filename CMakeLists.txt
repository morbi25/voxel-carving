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

# GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Open3D
option(GLIBCXX_USE_CXX11_ABI   "Set -D_GLIBCXX_USE_CXX11_ABI=1"       OFF)
option(STATIC_WINDOWS_RUNTIME  "Use static (MT/MTd) Windows runtime"  ON )

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No CMAKE_BUILD_TYPE specified, default to Release.")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Option 1: Use ExternalProject_Add, as shown in this CMake example.
# Option 2: Install Open3D first and use find_package, see
#           http://www.open3d.org/docs/release/cpp_project.html for details.
include(ExternalProject)
ExternalProject_Add(
    external_open3d
    PREFIX open3d
    GIT_REPOSITORY https://github.com/intel-isl/Open3D.git
    GIT_TAG master
    GIT_SHALLOW ON
    UPDATE_COMMAND ""
    # Check out https://github.com/intel-isl/Open3D/blob/master/CMakeLists.txt
    # For the full list of available options.
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DGLIBCXX_USE_CXX11_ABI=${GLIBCXX_USE_CXX11_ABI}
        -DSTATIC_WINDOWS_RUNTIME=${STATIC_WINDOWS_RUNTIME}
        -DBUILD_SHARED_LIBS=ON
        -DBUILD_PYTHON_MODULE=OFF
        -DBUILD_EXAMPLES=OFF
)

# Simulate importing Open3D::Open3D
ExternalProject_Get_Property(external_open3d INSTALL_DIR)
add_library(Open3DHelper INTERFACE)
add_dependencies(Open3DHelper external_open3d)
target_compile_features(Open3DHelper INTERFACE cxx_std_14)
target_compile_definitions(Open3DHelper INTERFACE _GLIBCXX_USE_CXX11_ABI=$<BOOL:${GLIBCXX_USE_CXX11_ABI}>)
target_include_directories(Open3DHelper INTERFACE "${INSTALL_DIR}/include" "${INSTALL_DIR}/include/open3d/3rdparty")
target_link_directories(Open3DHelper INTERFACE "${INSTALL_DIR}/lib")
target_link_libraries(Open3DHelper INTERFACE Open3D)
add_library(Open3D::Open3D ALIAS Open3DHelper)

# Set C++ flags
set(CMAKE_CXX_STANDARD 17)

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

set(HEADER_FILES 
    inc/Eigen.h
    inc/VoxelGrid.h
)

set(SOURCE_FILES 
    VoxelGrid.cpp
    calib_intrinsic.cpp
)

link_directories(${FreeImage_LIBRARY_DIR})
add_executable(voxel_carving main.cpp ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(voxel_carving ${OpenCV_LIBS} Eigen3::Eigen Open3D::Open3D)
target_include_directories(voxel_carving PUBLIC ${EIGEN3_INCLUDE_DIR})
target_sources(voxel_carving PRIVATE src/ImagePreprocessor.cpp src/PoseEstimator.cpp src/VoxelGrid.cpp)

# Unit Tests
add_executable(test_voxel_carving test/Test_ImagePreprocessor.cpp)
target_link_libraries(test_voxel_carving ${OpenCV_LIBS} Eigen3::Eigen gtest gtest_main)
target_include_directories(voxel_carving PUBLIC ${EIGEN3_INCLUDE_DIR})
add_test( test_voxel_carving test_voxel_carving )
target_sources(test_voxel_carving PRIVATE src/ImagePreprocessor.cpp src/PoseEstimator.cpp src/VoxelGrid.cpp)

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
endif()