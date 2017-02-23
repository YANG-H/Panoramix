# file (GLOB JLINKAGE_INCLUDE_FILES ./JLinkage/Include/bm/*.h ./JLinkage/Include/kdtree++/*.hpp ./JLinkage/Include/*.h)
file (GLOB JLINKAGE_SRC_FILES ./JLinkage/Src/*.h ./JLinkage/Src/*.cpp)
file (GLOB VPDETECTION_FILES ./*cpp ./*h)

include_directories (./JLinkage/Include)
include_directories (${OpenCV_INCLUDE_DIRS})
add_library ("vpdetection" STATIC ${JLINKAGE_SRC_FILES} ${VPDETECTION_FILES})
target_link_libraries ("vpdetection" ${OpenCV_LIBS})

set_property(TARGET "vpdetection" PROPERTY FOLDER "${PROJECT_NAME}.3rdparty")