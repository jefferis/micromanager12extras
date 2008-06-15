
########### next target ###############

SET(Nikon_LIB_SRCS
   Nikon.cpp
   Nikon.h
)

add_library(Nikon SHARED ${Nikon_LIB_SRCS})

target_link_libraries(Nikon MMDevice)

set_target_properties(Nikon PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS Nikon DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = Nikon.la
#Nikon_la_SOURCES = Nikon.cpp Nikon.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#Nikon_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#Nikon_la_LDFLAGS = -module
#
#install:
#	cp .libs/Nikon.so $(IJPATH)/$(MMPREFIX)Nikon
#
#EXTRA_DIST = Nikon.vcproj license.txt
#
