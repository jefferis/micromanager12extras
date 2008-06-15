
########### next target ###############

SET(LudlLow_LIB_SRCS
   Ludl.cpp
   Ludl.h
)

add_library(LudlLow SHARED ${LudlLow_LIB_SRCS})

target_link_libraries(LudlLow MMDevice)

set_target_properties(LudlLow PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS LudlLow DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = LudlLow.la
#LudlLow_la_SOURCES = Ludl.cpp Ludl.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#LudlLow_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#LudlLow_la_LDFLAGS = -module
#
#install:
#	cp .libs/LudlLow.so $(IJPATH)/$(MMPREFIX)LudlLow
#
###EXTRA_DIST = Ludl_vc8.vcproj Ludl.vcproj license.txt
#EXTRA_DIST = license.txt
