
########### next target ###############

SET(dc1394_LIB_SRCS
   dc1394.cpp
   dc1394.h
)

add_library(dc1394 SHARED ${dc1394_LIB_SRCS})

target_link_libraries(dc1394 MMDevice dc1394)

set_target_properties(dc1394 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS dc1394 DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive 
#lib_LTLIBRARIES = dc1394.la
#dc1394_la_SOURCES = dc1394.cpp dc1394.h 
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
###TODO: find the library in configure
#dc1394_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a /usr/local/lib/libdc1394.a
#dc1394_la_LDFLAGS = -module
#
#install:
#	cp .libs/dc1394.so $(IJPATH)/$(MMPREFIX)dc1394
#
#EXTRA_DIST = license.txt
