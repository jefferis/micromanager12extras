
########### next target ###############

SET(PVCAM_LIB_SRCS
   PVCAM.cpp
   PVCAM.h
   PVCAMUtils.cpp
   PVCAMUtils.h
   PVCAMUniversal.cpp
   Cascade.cpp
)

add_library(PVCAM SHARED ${PVCAM_LIB_SRCS})

target_link_libraries(PVCAM MMDevice)

set_target_properties(PVCAM PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS PVCAM DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = PVCAM.la
#PVCAM_la_SOURCES = PVCAM.cpp PVCAM.h PVCAMUtils.cpp PVCAMUtils.h PVCAMUniversal.cpp Cascade.cpp
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#PVCAM_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a $(LIBACE)
#PVCAM_la_LDFLAGS = -module $(PVCAMFRAMEWORKS)
#
#install:
#	cp .libs/PVCAM.so $(IJPATH)/$(MMPREFIX)PVCAM
#
#EXTRA_DIST = PVCAM.vcproj license.txt
