kinect_aux: main.c kinect_aux.h kinect_aux_lib.h
	gcc main.c -I /usr/include/libusb-1.0/ -lusb-1.0 -o kinect_aux

clean:
	rm -f kinect_aux
