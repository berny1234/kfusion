cmake_minimum_required(VERSION 2.8)

project(kfusion)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --ptxas-options=-v -arch compute_11 -code sm_11,sm_20,sm_21 -keep)

find_package(CUDA REQUIRED)
find_package(TooN REQUIRED)
find_package(OpenGL)
find_package(Freenect)
find_package(GLEW)
find_package(GLUT)
find_package(OpenNI)

include_directories(
	${CMAKE_BINARY_DIR}/include
	${TOON_INCLUDE_PATHS}
)

cuda_add_executable(devicequery
	devicequery.cpp
	)

set(kfusion_cuda_srcs
	helpers.cu
	helpers.h
	kfusion.cu
	kfusion.h
	minimal.cuh
	cutil_math.h
	)
	
cuda_add_library(kfusion STATIC
	${kfusion_cuda_srcs}
	perfstats.h
	OPTIONS
	-O3
	)

if (GLUT_FOUND AND FREENECT_FOUND AND GLEW_FOUND)
	include_directories(${FREENECT_INCLUDE_PATHS} ${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR})

	add_executable(kfusion_kinect
		kinect.cpp
		)
	target_link_libraries(kfusion_kinect kfusion ${FREENECT_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}) 

endif()


#if(GLUT_FOUND AND GLEW_FOUND)
#
#	include_directories(${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR})
#	add_executable(kfusion_test
#		test.cpp
#		)
#	
#	target_link_libraries(kfusion_test kfusion ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES}) 
#
#endif()

if(OPENNI_FOUND AND GLUT_FOUND)

find_package (Threads REQUIRED)

FIND_PATH( THREADS_PTHREADS_INCLUDE_DIR NAMES pthread.h 
    PATHS  THREADS_PTHREADS_INCLUDE_DIR )

include_directories (${THREADS_PTHREADS_INCLUDE_DIR})

FIND_LIBRARY( THREADS_PTHREADS_LIBRARY NAMES pthreadVC2
    PATHS THREADS_PTHREADS_LIBRARY
    )


message("OPENNI_FOUND")

	include_directories(${GLUT_INCLUDE_DIR} ${OPENNI_INCLUDE_DIRS})
	add_executable(openNIInterfaceTest
		openNIInterfaceTest.cpp 
		)
	target_link_libraries(openNIInterfaceTest ${OPENNI_LIBRARIES} ${GLUT_LIBRARIES}) 

	add_definitions(-DNOMINMAX)
	cuda_add_executable(kinectOpenNI
		kinectOpenNI.cpp ${kfusion_cuda_srcs} #for now
		)
	target_link_libraries(kinectOpenNI ${OPENNI_LIBRARIES} ${GLUT_LIBRARIES} ${THREADS_PTHREADS_LIBRARY}) 

endif()


