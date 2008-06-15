
########### next target ###############

SET(ZeissCAN29_LIB_SRCS
   ZeissCAN29.cpp
   ZeissCAN29.h
)

add_library(ZeissCAN29 SHARED ${ZeissCAN29_LIB_SRCS})

target_link_libraries(ZeissCAN29 MMDevice)

set_target_properties(ZeissCAN29 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS ZeissCAN29 DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = ZeissCAN29.la
#ZeissCAN29_la_SOURCES = ZeissCAN29.cpp ZeissCAN29.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#ZeissCAN29_la_LDFLAGS = -module
#ZeissCAN29_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#
#install:
#	cp .libs/ZeissCAN29.so $(IJPATH)/$(MMPREFIX)ZeissCAN29
