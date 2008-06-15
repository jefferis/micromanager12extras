
########### next target ###############

SET(Vincent_LIB_SRCS
   Vincent.cpp
   Vincent.h
)

add_library(Vincent SHARED ${Vincent_LIB_SRCS})

target_link_libraries(Vincent MMDevice)

set_target_properties(Vincent PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS Vincent DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = Vincent.la
#Vincent_la_SOURCES = Vincent.cpp Vincent.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#Vincent_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#Vincent_la_LDFLAGS = -module
#
#install:
#	cp .libs/Vincent.so $(IJPATH)/$(MMPREFIX)Vincent
#
#EXTRA_DIST =  Vincent.vcproj license.txt
