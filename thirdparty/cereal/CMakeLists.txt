cmake_minimum_required(VERSION 3.0.0)
project(CerealProxy C CXX)
include(ExternalProject)

ExternalProject_Add(cereal
     URL "file://${CMAKE_CURRENT_SOURCE_DIR}/cereal-master.zip"
     CMAKE_ARGS -DSKIP_SANDBOX:BOOL=OFF
     PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
     STAMP_DIR "${CMAKE_CURRENT_BINARY_DIR}"
# Disable install step
     INSTALL_COMMAND ""
)

# Specify include dir
ExternalProject_Get_Property(cereal source_dir)
set(CEREAL_INCLUDES "${source_dir}/include" PARENT_SCOPE)
