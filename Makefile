it: kinect_aux.cpp
	g++ kinect_aux.cpp -I /usr/include/libusb-1.0/ -lusb-1.0 -o kinect_aux

clean:
	rm -f kinect_aux
