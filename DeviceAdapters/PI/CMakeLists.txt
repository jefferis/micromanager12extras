
########### next target ###############

SET(PI_LIB_SRCS
   PI.cpp
   PI.h
)

add_library(PI SHARED ${PI_LIB_SRCS})

target_link_libraries(PI MMDevice)

set_target_properties(PI PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS PI DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = PI.la
#PI_la_SOURCES = PI.cpp PI.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#PI_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
#PI_la_LDFLAGS = -module
#
#install:
#	cp .libs/PI.so $(IJPATH)/$(MMPREFIX)PI
