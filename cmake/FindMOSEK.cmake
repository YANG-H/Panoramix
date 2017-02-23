# author: yanghao yangh2007@gmail.com
# ----------------------------------------------------------------------------
# versions - library suffixes

# known MOSEK versions, all entries have to be specified in descending order!
set (_MOSEK_VERSIONS_MAJOR 7 6)
set (_MOSEK_VERSIONS_MINOR 1 0)

# ----------------------------------------------------------------------------
# initialize search
if (NOT MOSEK_DIR)
  if (WIN32)
    foreach(_MOSEK_VER_MAJOR ${_MOSEK_VERSIONS_MAJOR})
      foreach(_MOSEK_VER_MINOR ${_MOSEK_VERSIONS_MINOR})
        if(NOT DEFINED MOSEK_DIR)
          get_filename_component (MOSEK_DIR 
            "[HKEY_LOCAL_MACHINE\\SOFTWARE\\mosek${_MOSEK_VER_MAJOR}${_MOSEK_VER_MINOR};InstallDir]" ABSOLUTE)
          set (MOSEK_VERSION_MAJOR ${_MOSEK_VER_MAJOR})
          set (MOSEK_VERSION_MINOR ${_MOSEK_VER_MINOR})
        elseif(${MOSEK_DIR} STREQUAL "")
          get_filename_component (MOSEK_DIR 
            "[HKEY_LOCAL_MACHINE\\SOFTWARE\\mosek${_MOSEK_VER_MAJOR}${_MOSEK_VER_MINOR};InstallDir]" ABSOLUTE)
          set (MOSEK_VERSION_MAJOR ${_MOSEK_VER_MAJOR})
          set (MOSEK_VERSION_MINOR ${_MOSEK_VER_MINOR})
        elseif(${MOSEK_DIR} STREQUAL "\registry")
          get_filename_component (MOSEK_DIR 
            "[HKEY_LOCAL_MACHINE\\SOFTWARE\\mosek${_MOSEK_VER_MAJOR}${_MOSEK_VER_MINOR};InstallDir]" ABSOLUTE)
          set (MOSEK_VERSION_MAJOR ${_MOSEK_VER_MAJOR})
          set (MOSEK_VERSION_MINOR ${_MOSEK_VER_MINOR})
        endif()
      endforeach()      
    endforeach()    
    #get_filename_component (MOSEK_DIR ${MOSEK_DIR} DIRECTORY)
  else ()
    set (MOSEK_DIR "$ENV{MOSEK_DIR}" CACHE PATH "Installation prefix for MOSEK." FORCE)
  endif()
endif ()

message ("MOSEK_DIR: " ${MOSEK_DIR})


# library name
set (MOSEK_LIBRARY_NAME "mosek")
if (NOT CMAKE_SIZE_OF_VOID_P EQUAL 4)
  set (MOSEK_LIBRARY_NAME "${MOSEK_LIBRARY_NAME}64")
endif ()
set (MOSEK_LIBRARY_NAME "${MOSEK_LIBRARY_NAME}_${MOSEK_VERSION_MAJOR}_${MOSEK_VERSION_MINOR}")

set (MOSEK_PLATFORM_DIR "${MOSEK_DIR}\\tools\\platform\\")

if (WIN32)
  set (MOSEK_PLATFORM_DIR "${MOSEK_PLATFORM_DIR}win")
elseif (APPLE)
  set (MOSEK_PLATFORM_DIR "${MOSEK_PLATFORM_DIR}osx")
else ()
  set (MOSEK_PLATFORM_DIR "${MOSEK_PLATFORM_DIR}linux")
endif ()

if (CMAKE_SIZE_OF_VOID_P EQUAL 4)
  set (MOSEK_PLATFORM_DIR "${MOSEK_PLATFORM_DIR}32")
else ()
  set (MOSEK_PLATFORM_DIR "${MOSEK_PLATFORM_DIR}64")
endif ()
set (MOSEK_PLATFORM_DIR "${MOSEK_PLATFORM_DIR}x86")

message ("MOSEK_PLATFORM_DIR: " ${MOSEK_PLATFORM_DIR})

set (MOSEK_BIN_DIR "${MOSEK_PLATFORM_DIR}\\bin\\")
set (MOSEK_INCLUDE_DIR "${MOSEK_PLATFORM_DIR}\\h\\")

message ("MOSEK_BIN_DIR: " ${MOSEK_BIN_DIR})
message ("MOSEK_INCLUDE_DIR: " ${MOSEK_INCLUDE_DIR})


message ("MOSEK_LIBRARY_NAME: " ${MOSEK_LIBRARY_NAME})

 find_library (
      MOSEK_LIBRARY
        NAMES         ${MOSEK_LIBRARY_NAME}
        HINTS         "${MOSEK_BIN_DIR}"
        DOC           "MOSEK link library."
        NO_DEFAULT_PATH
    )

mark_as_advanced (MOSEK_INCLUDE_DIR)
mark_as_advanced (MOSEK_BIN_DIR)
mark_as_advanced (MOSEK_LIBRARY)
# ----------------------------------------------------------------------------
# prerequisite libraries
set (MOSEK_INCLUDES  "${MOSEK_INCLUDE_DIR}")
set (MOSEK_LIBRARIES "${MOSEK_LIBRARY}")

# ----------------------------------------------------------------------------
# aliases / backwards compatibility
set (MOSEK_INCLUDE_DIRS "${MOSEK_INCLUDES}")


# ----------------------------------------------------------------------------
# handle the QUIETLY and REQUIRED arguments and set *_FOUND to TRUE
# if all listed variables are found or TRUE
include (FindPackageHandleStandardArgs)

set (MOSEK_REQUIRED_VARS
  MOSEK_DIR
  MOSEK_INCLUDE_DIR
  MOSEK_LIBRARY
  MOSEK_BIN_DIR
)


find_package_handle_standard_args (
  MOSEK
# MESSAGE
    DEFAULT_MSG
# VARIABLES
    ${MOSEK_REQUIRED_VARS}
)