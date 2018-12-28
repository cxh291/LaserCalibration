cmake_minimum_required(VERSION 2.8)
STRING( REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} )  
set(PROJECT_NAME ${CURRENT_FOLDER})
project(${PROJECT_NAME})

include_directories(${PROJECT_SOURCE_DIR}/src)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} )

#opencv
find_package(OpenCV REQUIRED)
list(APPEND 3RD_LIBS ${OpenCV_LIBS})
include_directories(${OpenCV_INCLUDE_DIRS} )

#Qt4
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui QtNetwork REQUIRED)
INCLUDE(${QT_USE_FILE})
list(APPEND 3RD_LIBS ${QT_LIBRARIES})

#openGL
find_package( OpenGL REQUIRED )
list(APPEND 3RD_LIBS ${OPENGL_gl_LIBRARY})

#CC 2.6.3.1
#set(CC_DIR ${PROJECT_SOURCE_DIR}/3rdParty/CC-2.6.3.1)
find_path(CC_DIR "")
IF(CC_DIR)
	MESSAGE(STATUS "Found CC_DIR: ${CC_DIR}")
	include_directories(${CC_DIR}/CCCORE/include)
	include_directories(${CC_DIR}/CCFbo/include)
	include_directories(${CC_DIR}/Glew/include)
	include_directories(${CC_DIR}/qCC_db/include)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/qCC_db/lib/QCC_DB_LIBd.lib optimized ${CC_DIR}/qCC_db/lib/QCC_DB_LIB.lib)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/CCCORE/lib/CC_CORE_LIBd.lib optimized ${CC_DIR}/CCCORE/lib/CC_CORE_LIB.lib)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/CCFbo/lib/CC_FBO_LIBd.lib optimized ${CC_DIR}/CCFbo/lib/CC_FBO_LIB.lib)
	list(APPEND 3RD_LIBS debug ${CC_DIR}/Glew/lib/GLEW_LIBd.lib optimized ${CC_DIR}/Glew/lib/GLEW_LIB.lib)
ELSE(CC_DIR)
	MESSAGE(FATAL_ERROR "Could not find CC")
ENDIF(CC_DIR)

#COMMON_GQ
include_directories(${PROJECT_SOURCE_DIR}/src/Common_GQ)
file(GLOB_RECURSE COMMON_GQ_FILES ${PROJECT_SOURCE_DIR}/src/Common_GQ/*.cpp)

#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(APPEND 3RD_LIBS ${PCL_LIBRARIES})
add_definitions(-DNOMINMAX)

#ExtrinsicCalibration lib
file(GLOB ExtrinsicCalibration_FILES ${PROJECT_SOURCE_DIR}/src/ExtrinsicCalibration/*.*)
add_library(ExtrinsicCalibration ${ExtrinsicCalibration_FILES})
target_link_libraries(ExtrinsicCalibration ${3RD_LIBS} )

#LaserLaserCalibByLinear
file(GLOB LaserLaserCalibByLinear_FILES ${COMMON_GQ_FILES} ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/*.* ${PROJECT_SOURCE_DIR}/ui/LaserLaserCalibByLinear/*.*)
file(GLOB moc_list ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/PointCloudWindow.h ${PROJECT_SOURCE_DIR}/ui/CCPointCloudWindow/ccGLWindow.h ${PROJECT_SOURCE_DIR}/ui/LaserLaserCalibByLinear/*.h)
QT4_WRAP_CPP(MOCS ${moc_list})
QT4_ADD_RESOURCES(QRCS ${PROJECT_SOURCE_DIR}/resource/resource.qrc)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
add_executable(LaserLaserCalibByLinear ${LaserLaserCalibByLinear_FILES} ${MOCS} ${QRCS})
target_link_libraries(LaserLaserCalibByLinear ${3RD_LIBS} ExtrinsicCalibration)


#test exe
file(GLOB_RECURSE TEST_FILES ${COMMON_GQ_FILES} ${PROJECT_SOURCE_DIR}/src/main.cpp) 
add_executable(${PROJECT_NAME}_TEST ${TEST_FILES})
target_link_libraries(${PROJECT_NAME}_TEST ${3RD_LIBS})
