
########### next target ###############

SET(ASIFW1000_LIB_SRCS
   ASIFW1000.cpp
   ASIFW1000.h
   ASIFW1000Hub.cpp
   ASIFW1000Hub.h
)

add_library(ASIFW1000 SHARED ${ASIFW1000_LIB_SRCS})

target_link_libraries(ASIFW1000 MMDevice)

set_target_properties(ASIFW1000 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS ASIFW1000 DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = ASIFW1000.la
#ASIFW1000_la_SOURCES = ASIFW1000.cpp ASIFW1000.h ASIFW1000Hub.cpp ASIFW1000Hub.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#ASIFW1000_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#ASIFW1000_la_LDFLAGS = -module
#
#install:
#	cp .libs/ASIFW1000.so $(IJPATH)/$(MMPREFIX)ASIFW1000
#
#EXTRA_DIST = ASIFW1000.vcproj license.txt
