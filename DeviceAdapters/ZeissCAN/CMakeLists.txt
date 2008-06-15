
########### next target ###############

SET(ZeissCAN_LIB_SRCS
   ZeissCAN.cpp
   ZeissCAN.h
)

add_library(ZeissCAN SHARED ${ZeissCAN_LIB_SRCS})

target_link_libraries(ZeissCAN MMDevice)

set_target_properties(ZeissCAN PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS ZeissCAN DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = ZeissCAN.la
#ZeissCAN_la_SOURCES = ZeissCAN.cpp ZeissCAN.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#ZeissCAN_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#ZeissCAN_la_LDFLAGS = -module
#
#install:
#	cp .libs/ZeissCAN.so $(IJPATH)/$(MMPREFIX)ZeissCAN
