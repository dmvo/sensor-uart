include ../Makefile.include

OBJECTS = \
	       ble_advdata.o \
	       ble_conn_params.o \
	       ble_debug_assert_handler.o \
	       ble_hrs.o \
	       ble_bas.o \
	       ble_srv_common.o \
	       ble_hts.o \

libs:	libble.a($(OBJECTS))

clean:
	-rm -rf lib*.a *.o *~
