
########### next target ###############

SET(DemoCamera_LIB_SRCS
   DemoCamera.cpp
   DemoCamera.h
)

add_library(DemoCamera SHARED ${DemoCamera_LIB_SRCS})

target_link_libraries(DemoCamera MMDevice)

set_target_properties(DemoCamera PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS DemoCamera DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive -fweak
#lib_LTLIBRARIES = DemoCamera.la
#DemoCamera_la_SOURCES = DemoCamera.cpp DemoCamera.h
#DemoCamera_la_LDFLAGS = -module 
#DemoCamera_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
#
#install:
#	cp .libs/DemoCamera.so $(IJPATH)/$(MMPREFIX)DemoCamera
#
#EXTRA_DIST = DemoCamera.vcproj license.txt
