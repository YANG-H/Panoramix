set_property(GLOBAL PROPERTY USE_FOLDERS On)

set(CMAKE_AUTOMOC on)
set(CMAKE_AUTOUIC on)
set(CMAKE_AUTORCC on)

file (GLOB panoramix_sources 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu" 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cuh"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.qrc"
)
file (GLOB panoramix_test_sources 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.test.cpp" 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.test.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.test.cu" 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.test.cuh"
)
if (panoramix_test_sources)
    list (REMOVE_ITEM panoramix_sources ${panoramix_test_sources})
endif()
source_group ("Sources" FILES ${panoramix_sources})
source_group ("Sources" FILES ${panoramix_test_sources})

file (GLOB panoramix_experimental_sources 
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.cpp" 
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.cu" 
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.cuh"
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.qrc"
)
file (GLOB panoramix_experimental_test_sources 
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.test.cpp" 
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.test.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.test.cu" 
    "${CMAKE_CURRENT_SOURCE_DIR}/experimental/*.test.cuh"
)
if (panoramix_experimental_test_sources)
    list (REMOVE_ITEM panoramix_experimental_sources 
        ${panoramix_experimental_test_sources} )
endif()
source_group ("Sources Experimental" FILES ${panoramix_experimental_sources})
source_group ("Sources Experimental" FILES ${panoramix_experimental_test_sources})


# the lib project
message(STATUS "panoramix_sources:")
foreach(i ${panoramix_sources})
    message (STATUS ${i})  
endforeach()
message(STATUS "panoramix_experimental_sources:")
foreach(i ${panoramix_experimental_sources})
    message (STATUS ${i})  
endforeach()

panoramix_add_library(Panoramix 
    ${panoramix_sources} 
    ${panoramix_experimental_sources} pch.hpp pch.cpp)
target_include_directories(Panoramix PUBLIC ${DEPENDENCY_INCLUDES} 
    ${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src 
    ${CMAKE_CURRENT_SOURCE_DIR}/experimental)
target_link_libraries (Panoramix ${DEPENDENCY_LIBS})
add_dependencies(Panoramix ${DEPENDENCY_NAMES})

include(../cmake/PrecompiledHeader.cmake)
add_precompiled_header(Panoramix "pch.hpp" SOURCE_CXX "pch.cpp")

# the test project
message(STATUS "panoramix_test_sources:")
foreach(i ${panoramix_test_sources})
    message (STATUS ${i})  
endforeach()
enable_testing ()
panoramix_add_executable(Panoramix.UnitTest 
    ${panoramix_test_sources} 
    ${panoramix_experimental_test_sources}
    ./panoramix.unittest.hpp ./panoramix.unittest.cpp)
target_include_directories(Panoramix.UnitTest PUBLIC ${DEPENDENCY_INCLUDES}
    ${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src 
    ${CMAKE_CURRENT_SOURCE_DIR}/experimental)
target_link_libraries (Panoramix.UnitTest Panoramix)
target_link_libraries (Panoramix.UnitTest ${DEPENDENCY_LIBS})
add_dependencies(Panoramix.UnitTest Panoramix)
add_dependencies(Panoramix.UnitTest ${DEPENDENCY_NAMES})