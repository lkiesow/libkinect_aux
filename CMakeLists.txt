cmake_minimum_required(VERSION 2.8)

PROJECT(libkinect_aux)


###########################################
# find: libusb include dir
###########################################
find_path(LIBUSB-1.0-INCLUDE-DIR
	NAMES
		libusb-1.0/libusb.h
	PATHS
		/usr/include
		/usr/local/include
		/opt/local/include
	PATH_SUFFIXES
		libusb-1.0
)

if(LIBUSB-1.0-INCLUDE-DIR)
	INCLUDE_DIRECTORIES(${LIBUSB-1.0-INCLUDE-DIR}/libusb-1.0)
	message(">> found libusb-1.0 includes at '" ${LIBUSB-1.0-INCLUDE-DIR} "'")
endif(LIBUSB-1.0-INCLUDE-DIR)


###########################################
# find: libusb library
###########################################
FIND_LIBRARY(LIBUSB-1.0
	NAMES
		usb-1.0
	PATHS
		/usr/lib/
		/usr/local/lib/
)

if(LIBUSB-1.0)
	message(">> found libusb-1.0 library at '" ${LIBUSB-1.0} "'")
endif(LIBUSB-1.0)


###########################################
# kinect_aux library
###########################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L ${LIBUSB-1.0} -lusb-1.0 -lm ")
ADD_LIBRARY(kinect_aux SHARED src/kinect_aux.c)
TARGET_LINK_LIBRARIES(kinect_aux ${LIBUSB-1.0})


###########################################
# test executable
###########################################
ADD_EXECUTABLE(test src/test)
TARGET_LINK_LIBRARIES(test kinect_aux)
