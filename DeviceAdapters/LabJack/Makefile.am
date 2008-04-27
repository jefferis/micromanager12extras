AM_CXXFLAGS = -fpermissive 
lib_LTLIBRARIES = LabJack.la
LabJack_la_SOURCES = LabJack.cpp LabJack.h u3.h u3.c 
## adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
##TODO: find the library in configure
LabJack_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a /usr/local/lib/liblabjackusb.dylib 
LabJack_la_LDFLAGS = -module

install:
	cp .libs/LabJack.so $(IJPATH)/$(MMPREFIX)LabJack

EXTRA_DIST = license.txt
