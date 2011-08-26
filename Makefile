
include config.mk

lib: kinect_aux.c kinect_aux.h
	gcc $(FLAGS) $(LIBUSBINC) $(USBLIB) $(MATHLIB) -c -fPIC kinect_aux.c -o kinect_aux.o
	gcc -shared -Wl,-soname,libkinect_aux.so -o libkinect_aux.so kinect_aux.o

test: test.c lib
	gcc test.c $(USBLIB) $(KINECTLIB) $(MATHLIB) -o test

install: lib
	cp libkinect_aux.so $(INSTDIRL)
	cp kinect_aux.h $(INSTDIRI)

uninstall: 
	rm -f $(INSTDIRL)libkinect_aux.so
	rm -f $(INSTDIRI)kinect_aux.h

clean:
	rm -f test kinect_aux.o libkinect_aux.so
