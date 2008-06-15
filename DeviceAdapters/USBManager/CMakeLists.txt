
########### next target ###############

SET(USBManager_LIB_SRCS
   USBManager.cpp
   USBManager.h
)

add_library(USBManager SHARED ${USBManager_LIB_SRCS})

target_link_libraries(USBManager MMDevice)

set_target_properties(USBManager PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS USBManager DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = USBManager.la
#USBManager_la_SOURCES = USBManager.cpp USBManager.h 
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#USBManager_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a  $(LIBUSB)
#USBManager_la_LDFLAGS = -module
#
#install:
#	cp .libs/USBManager.so $(IJPATH)/$(MMPREFIX)USBManager
#
#EXTRA_DIST = license.txt
