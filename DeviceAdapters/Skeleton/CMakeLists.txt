
########### next target ###############

SET(Skeleton_LIB_SRCS
   Skeleton.cpp
   Skeleton.h
)

add_library(Skeleton SHARED ${Skeleton_LIB_SRCS})

target_link_libraries(Skeleton)

set_target_properties(Skeleton PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS Skeleton DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = Skeleton.la
#Skeleton_la_SOURCES = Skeleton.cpp Skeleton.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#Skeleton__la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
#Skeleton_la_LDFLAGS = -module
#
#install:
#	cp .libs/Skeleton.so $(IJPATH)/$(MMPREFIX)Skeleton
#
#EXTRA_DIST = license.txt
