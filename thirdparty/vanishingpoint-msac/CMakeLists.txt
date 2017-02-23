
set (SOURCE errorNIETO.h errorNIETO.cpp MSAC.h MSAC.cpp lmcurve.c lmcurve.h lmmin.c lmmin.h lmstruct.h)
find_package (OpenCV)
include_directories(	
	${OPENCV_INCLUDE_DIR}
)
include_directories (${OpenCV_INCLUDE_DIRS})
add_library ("vanishingpoint-msac" STATIC ${SOURCE})
TARGET_LINK_LIBRARIES( vanishingpoint-msac ${OpenCV_LIBS})

set_property(TARGET "vanishingpoint-msac" PROPERTY FOLDER "${PROJECT_NAME}.3rdparty")