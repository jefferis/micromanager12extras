
########### next target ###############

SET(DemoStreamingCamera_LIB_SRCS
   DemoStreamingCamera.cpp
   DemoStreamingCamera.h
   NoiseProcessor.cpp
   SignalGenerator.cpp
)

add_library(DemoStreamingCamera SHARED ${DemoStreamingCamera_LIB_SRCS})

target_link_libraries(DemoStreamingCamera MMDevice)

set_target_properties(DemoStreamingCamera PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS DemoStreamingCamera DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive -fweak
#lib_LTLIBRARIES = DemoStreamingCamera.la
#DemoStreamingCamera_la_SOURCES = DemoStreamingCamera.cpp DemoStreamingCamera.h NoiseProcessor.cpp SignalGenerator.cpp
#DemoStreamingCamera_la_LDFLAGS = -module 
#DemoStreamingCamera_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a $(LIBACE)
#
#install:
#	cp .libs/DemoStreamingCamera.so $(IJPATH)/$(MMPREFIX)DemoStreamingCamera
#
#EXTRA_DIST = DemoStreamingCamera.vcproj license.txt
