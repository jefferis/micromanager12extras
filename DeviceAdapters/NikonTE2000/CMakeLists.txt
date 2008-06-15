
########### next target ###############

SET(NikonTE2000_LIB_SRCS
   NikonTE2000.cpp
   NikonTE2000.h
   TEHub.cpp
   TEHub.h
)

add_library(NikonTE2000 SHARED ${NikonTE2000_LIB_SRCS})

target_link_libraries(NikonTE2000 MMDevice)

set_target_properties(NikonTE2000 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS NikonTE2000 DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = NikonTE2000.la
#NikonTE2000_la_SOURCES = NikonTE2000.cpp NikonTE2000.h TEHub.cpp TEHub.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#NikonTE2000_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#NikonTE2000_la_LDFLAGS = -module
#
#install:
#	cp .libs/NikonTE2000.so $(IJPATH)/$(MMPREFIX)NikonTE2000
#
#EXTRA_DIST = NikonTE2000.vcproj NikonTE2000_vc8.sln license.txt
