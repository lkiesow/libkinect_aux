it: kinect_aux.c
	gcc kinect_aux.c -I /usr/include/libusb-1.0/ -lusb-1.0 -o kinect_aux

clean:
	rm -f kinect_aux
