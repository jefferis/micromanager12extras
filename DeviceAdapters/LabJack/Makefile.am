AM_CXXFLAGS = -fpermissive 
lib_LTLIBRARIES = LabJack.la
LabJack_la_SOURCES = LabJack.cpp LabJack.h u3.h u3.c 
## adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
##TODO: find the library in configure
#LabJack_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a /usr/lib/liblabjackusb.dylib 
LabJack_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
# LabJack_la_LDADD = /usr/lib/liblabjackusb.dylib

#LabJack_la_LDFLAGS = -module
LabJack_la_LDFLAGS = -module /usr/lib/liblabjackusb.dylib


install:
	cp .libs/LabJack.so $(IJPATH)/$(MMPREFIX)LabJack

EXTRA_DIST = license.txt
