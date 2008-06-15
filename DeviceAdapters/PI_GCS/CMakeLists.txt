
########### next target ###############

SET(PIGCS_LIB_SRCS
   PI_GCS.cpp
   PI_GCS.h
)

add_library(PIGCS SHARED ${PIGCS_LIB_SRCS})

target_link_libraries(PIGCS MMDevice)

set_target_properties(PIGCS PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS PIGCS DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = PIGCS.la
#PIGCS_la_SOURCES = PI_GCS.cpp PI_GCS.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#PIGCS_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
#PIGCS_la_LDFLAGS = -module
#
#install:
#	cp .libs/PIGCS.so $(IJPATH)/$(MMPREFIX)PIGCS
