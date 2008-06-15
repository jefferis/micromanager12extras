
########### next target ###############

SET(Ludl_LIB_SRCS
   Ludl.cpp
   Ludl.h
)

add_library(Ludl SHARED ${Ludl_LIB_SRCS})

target_link_libraries(Ludl MMDevice)

set_target_properties(Ludl PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS Ludl DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = Ludl.la
#Ludl_la_SOURCES = Ludl.cpp Ludl.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#Ludl_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#Ludl_la_LDFLAGS = -module
#
#install:
#	cp .libs/Ludl.so $(IJPATH)/$(MMPREFIX)Ludl
#
###EXTRA_DIST = Ludl_vc8.vcproj Ludl.vcproj license.txt
#EXTRA_DIST = license.txt
