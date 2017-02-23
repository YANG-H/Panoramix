cmake_minimum_required (VERSION 3.0)
project (Panoramix VERSION 0.1 LANGUAGES CXX)

set (CMAKE_ALLOW_LOOSE_CONSTRUCTS true)
list (APPEND CMAKE_MODULE_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ 
)

if (NOT DEFINED MSVC)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
endif ()

set (TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/testdata/" CACHE PATH "path to test data")
set (CACHE_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cache/" CACHE PATH "path to cache data")
set (MATLAB_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../MATLABTools/" CACHE PATH "path to matlab code")


set (DEPENDENCY_NAMES "")
set (DEPENDENCY_INCLUDES "")
set (DEPENDENCY_LIBS "")
set (DEPENDENCY_BIN_PATHS "")


# add opencv
find_package(OpenCV REQUIRED)
list (APPEND DEPENDENCY_INCLUDES ${OpenCV_INCLUDE_DIRS})
list (APPEND DEPENDENCY_LIBS ${OpenCV_LIBS})
if (DEFINED _OpenCV_LIB_PATH)
    list (APPEND DEPENDENCY_BINS_PATHS ${_OpenCV_LIB_PATH})
endif ()

# add matlab
find_package(MATLAB REQUIRED)
#  MATLAB_INCLUDE_DIR: include path for mex.h, engine.h
#  MATLAB_LIBRARIES:   required libraries: libmex, etc
#  MATLAB_MEX_LIBRARY: path to libmex.lib
#  MATLAB_MX_LIBRARY:  path to libmx.lib
#  MATLAB_MAT_LIBRARY:  path to libmat.lib # added
#  MATLAB_ENG_LIBRARY: path to libeng.lib
#  MATLAB_ROOT: path to Matlab's root directory
list (APPEND DEPENDENCY_INCLUDES ${MATLAB_INCLUDE_DIR})
list (APPEND DEPENDENCY_LIBS ${MATLAB_LIBRARIES})
list (APPEND DEPENDENCY_LIBS ${MATLAB_MAT_LIBRARY})

# add_qt
set (Qt_DIR "" CACHE PATH "Qt root directory here")
set (Qt_MODULES_REQUIRED Core Gui Widgets OpenGL)
list (APPEND CMAKE_PREFIX_PATH ${Qt_DIR})
set(CMAKE_INCLUDE_CURRENT_DIR on)
set (Qt_PATH ${Qt_DIR}/bin)
set (Qt_INCLUDES "")
set (Qt_LIBS "")
foreach (_M ${Qt_MODULES_REQUIRED})
    find_package ("Qt5${_M}" REQUIRED)
    string(COMPARE EQUAL ${_M} OpenGL _IsOpenGL)
    if(${_IsOpenGL})
        find_package (OPENGL REQUIRED)
        list(APPEND Qt_INCLUDES ${OPENGL_INCLUDE_DIR})
        list(APPEND Qt_LIBS ${OPENGL_LIBRARIES})
    endif()
    list(APPEND Qt_LIBS "Qt5::${_M}")
endforeach()
list (APPEND DEPENDENCY_INCLUDES ${Qt_INCLUDES})
list (APPEND DEPENDENCY_LIBS ${Qt_LIBS})
list (APPEND DEPENDENCY_BINS_PATHS ${Qt_PATH})
if (MSVC)
    message (WARNING 
        "do not forget to add QT_QPA_PLATFORM_PLUGIN_PATH=${Qt_DIR}/plugins/platforms to env")
endif ()

message (STATUS "test data directory: " ${TEST_DATA_DIR})
message (STATUS "default cache data directory: " ${CACHE_DATA_DIR})
message (STATUS "matlab code directory: " ${MATLAB_CODE_DIR})
if (MSVC)
    add_definitions ( "/D_USE_MATH_DEFINES")
    add_definitions ("/DNOMINMAX")
    add_definitions ( "/DPANORAMIX_TEST_DATA_DIR_STR=\"${TEST_DATA_DIR}\"" )
    add_definitions ( "/DPANORAMIX_CACHE_DATA_DIR_STR=\"${CACHE_DATA_DIR}\"" )
    add_definitions ( "/DPANORAMIX_MATLAB_CODE_DIR_STR=\"${MATLAB_CODE_DIR}\"" )
else ()
    add_definitions ( "-DPANORAMIX_TEST_DATA_DIR_STR=\"${TEST_DATA_DIR}\"" )
    add_definitions ( "-DPANORAMIX_CACHE_DATA_DIR_STR=\"${CACHE_DATA_DIR}\"" )
    add_definitions ( "-DPANORAMIX_MATLAB_CODE_DIR_STR=\"${MATLAB_CODE_DIR}\"" )
endif ()


# include third party codes
add_subdirectory ("thirdparty")
list (APPEND DEPENDENCY_NAMES ${ThirdParty_NAMES})
list (APPEND DEPENDENCY_INCLUDES ${ThirdParty_INCLUDES})
list (APPEND DEPENDENCY_LIBS ${ThirdParty_LIBS})

list (APPEND DEPENDENCY_INCLUDES ".")


# panoramix_add_executable
macro(panoramix_add_executable BIN_NAME)
    if (CUDA_TOOLKIT_INCLUDE)
        cuda_add_executable(${BIN_NAME} ${ARGN})
    else()
        add_executable(${BIN_NAME} ${ARGN})
    endif()
endmacro()
# panoramix_add_library
macro(panoramix_add_library BIN_NAME)
    if (CUDA_TOOLKIT_INCLUDE)
        cuda_add_library(${BIN_NAME} ${ARGN})
    else()
        add_library(${BIN_NAME} ${ARGN})
    endif()
endmacro()



# the libs
add_subdirectory (panoramix)


# executable
add_subdirectory (executable)


