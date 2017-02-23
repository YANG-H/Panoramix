add_subdirectory ("lsd_1.6")
add_subdirectory ("slic")
add_subdirectory ("vanishingpoint-msac")
add_subdirectory ("vpdetection")
#add_subdirectory ("wclique")

set (_ThirdParty_INCLUDES 
	${CMAKE_CURRENT_SOURCE_DIR}/eigen
	${CMAKE_CURRENT_SOURCE_DIR}/cereal/include
	${CMAKE_CURRENT_SOURCE_DIR}/lsd_1.6
	${CMAKE_CURRENT_SOURCE_DIR}/slic
	${CMAKE_CURRENT_SOURCE_DIR}/vanishingpoint-msac
	${CMAKE_CURRENT_SOURCE_DIR}/vpdetection
	#${CMAKE_CURRENT_SOURCE_DIR}/wclique
	${CMAKE_CURRENT_SOURCE_DIR}/misc)

set (_ThirdParty_LIBS 
	"lsd_1.6" 
	"slic" 
	"vanishingpoint-msac" 
	"vpdetection" 
	#"wclique"
)

set (_ThirdParty_NAMES "")

add_subdirectory("cereal")
list (APPEND _ThirdParty_NAMES "cereal")
list (APPEND _ThirdParty_INCLUDES ${CEREAL_INCLUDES})

add_subdirectory("eigen3")
list (APPEND _ThirdParty_NAMES "eigen3")
list (APPEND _ThirdParty_INCLUDES ${EIGEN3_INCLUDES})

add_subdirectory("googletest")
list (APPEND _ThirdParty_NAMES "googletest")
list (APPEND _ThirdParty_INCLUDES ${GTEST_INCLUDES})
list (APPEND _ThirdParty_LIBS ${GTEST_LIBS})

set (ThirdParty_NAMES ${_ThirdParty_NAMES} PARENT_SCOPE)
set (ThirdParty_INCLUDES ${_ThirdParty_INCLUDES} PARENT_SCOPE)
set (ThirdParty_LIBS ${_ThirdParty_LIBS} PARENT_SCOPE)