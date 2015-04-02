include ../Makefile.include

OBJECTS = \
	       app_timer.o \
	       simple_uart.o \
	       pstorage.o \

libs:	libapp.a($(OBJECTS))

clean:
	-rm -rf lib*.a *.o *~
