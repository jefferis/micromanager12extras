
########### next target ###############

SET(Prior_LIB_SRCS
   Prior.cpp
   prior.h
)

add_library(Prior SHARED ${Prior_LIB_SRCS})

target_link_libraries(Prior MMDevice)

set_target_properties(Prior PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS Prior DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = Prior.la
#Prior_la_SOURCES = Prior.cpp prior.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#Prior_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#Prior_la_LDFLAGS = -module
#
#install:
#	cp .libs/Prior.so $(IJPATH)/$(MMPREFIX)Prior
#
#EXTRA_DIST = Prior.vcproj license.txt
