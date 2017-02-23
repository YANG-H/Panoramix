
set (SOURCE lsd.h lsd.c)

source_group ("lsd_1.6" FILES ${SOURCE})
add_library ("lsd_1.6" STATIC ${SOURCE})

set_property(TARGET "lsd_1.6" PROPERTY FOLDER "${PROJECT_NAME}.3rdparty")