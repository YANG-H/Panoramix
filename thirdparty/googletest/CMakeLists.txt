cmake_minimum_required(VERSION 3.0.0)
project(GoogleTestProxy C CXX)
include(ExternalProject)

ExternalProject_Add(googletest
     URL "file://${CMAKE_CURRENT_SOURCE_DIR}/googletest-master.zip"
     CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
               -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
               -Dgtest_force_shared_crt=ON
               -DBUILD_GTEST=ON
     PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
     STAMP_DIR "${CMAKE_CURRENT_BINARY_DIR}"
# Disable install step
     INSTALL_COMMAND ""
)

# Specify include dir
ExternalProject_Get_Property(googletest source_dir)
set(GTEST_INCLUDES ${source_dir}/googletest/include PARENT_SCOPE)

# Specify gtest libs
ExternalProject_Get_Property(googletest binary_dir)
set(GTEST_LIBS_DIR ${binary_dir}/googlemock/gtest)
if(NOT MSVC)
    set(GTEST_LIBS 
        "${GTEST_LIBS_DIR}/libgtest.a ${GTEST_LIBS_DIR}/libgtest_main.a" 
        PARENT_SCOPE)
else()
    set(GTEST_LIBS
        debug "${GTEST_LIBS_DIR}/DebugLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_FIND_LIBRARY_SUFFIXES}"
        optimized "${GTEST_LIBS_DIR}/ReleaseLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_FIND_LIBRARY_SUFFIXES}"
        debug "${GTEST_LIBS_DIR}/DebugLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_FIND_LIBRARY_SUFFIXES}"
        optimized "${GTEST_LIBS_DIR}/ReleaseLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_FIND_LIBRARY_SUFFIXES}"
        PARENT_SCOPE)
endif()