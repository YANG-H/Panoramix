
set (SOURCE SLIC.h SLIC.cpp)

source_group ("slic" FILES ${SOURCE})
add_library ("slic" STATIC ${SOURCE})

set_property(TARGET "slic" PROPERTY FOLDER "${PROJECT_NAME}.3rdparty")