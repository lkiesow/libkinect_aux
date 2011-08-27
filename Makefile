
include config.mk

lib: $(SRCDIR)kinect_aux.c $(SRCDIR)kinect_aux.h
	mkdir -p $(OBJDIR)
	gcc $(FLAGS) $(LIBUSBINC) $(USBLIB) $(MATHLIB) -c -fPIC \
		$(SRCDIR)kinect_aux.c -o $(OBJDIR)kinect_aux.o
	gcc -shared -Wl,-soname,libkinect_aux.so -o libkinect_aux.so \
		$(OBJDIR)kinect_aux.o

test: $(SRCDIR)test.c lib
	gcc $(SRCDIR)test.c $(USBLIB) $(KINECTLIB) $(MATHLIB) -o test

doc: Doxyfile $(SRCDIR)kinect_aux.h
	doxygen Doxyfile

all: lib test doc

install: lib
	cp libkinect_aux.so $(INSTDIRL)
	cp $(SRCDIR)kinect_aux.h $(INSTDIRI)

uninstall: 
	rm -f $(INSTDIRL)libkinect_aux.so
	rm -f $(INSTDIRI)kinect_aux.h

clean:
	rm -f test libkinect_aux.so
	rm -rf obj/ doc/
