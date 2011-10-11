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
FIND_PACKAGE(Threads)
ADD_LIBRARY(kinect_aux SHARED src/kinect_aux.c)
TARGET_LINK_LIBRARIES(kinect_aux ${LIBUSB-1.0} ${CMAKE_THREAD_LIBS_INIT} "m" )


###########################################
# test executable
###########################################
ADD_EXECUTABLE(test src/test)
TARGET_LINK_LIBRARIES(test kinect_aux)


###########################################
# install
###########################################
install( 
	FILES ${CMAKE_CURRENT_BINARY_DIR}/libkinect_aux.so
	DESTINATION lib )
install( 
	FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/kinect_aux.h
	DESTINATION include )
#install( 
#	FILES ${rgbPcGrabber_SOURCE_DIR}/rgbPcGrabber.1 
#	DESTINATION share/man/man1 )


###########################################
# uninstall
###########################################
add_custom_target( uninstall 
	COMMENT "Removing installed files"
	COMMAND "echo" "@CMAKE_INSTALL_PREFIX@/lib64/libkinect_aux.so"
	COMMAND "rm" "@CMAKE_INSTALL_PREFIX@/lib64/libkinect_aux.so"
	COMMAND "echo" "@CMAKE_INSTALL_PREFIX@/include/kinect_aux.h"
	COMMAND "rm" "@CMAKE_INSTALL_PREFIX@/include/kinect_aux.h" )


###########################################
# Generate documentation
###########################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	add_custom_target( doc
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM
	)
endif(DOXYGEN_FOUND)
