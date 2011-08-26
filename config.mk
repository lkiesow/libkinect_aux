INSTDIRL =/usr/lib64/
INSTDIRI =/use/include/
LIBUSBINC=${shell pkg-config --cflags libusb-1.0}
USBLIB=${shell pkg-config --libs libusb-1.0}
MATHLIB  =-lm
KINECTLIB=-L . -lkinect_aux

COMPILER = gcc
FLAGS    = -Wall
