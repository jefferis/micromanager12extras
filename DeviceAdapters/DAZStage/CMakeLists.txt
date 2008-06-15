
########### next target ###############

SET(DAZStage_LIB_SRCS
   DAZStage.cpp
   DAZStage.h
)

add_library(DAZStage SHARED ${DAZStage_LIB_SRCS})

target_link_libraries(DAZStage MMDevice)

set_target_properties(DAZStage PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS DAZStage DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = DAZStage.la
#DAZStage_la_SOURCES = DAZStage.cpp DAZStage.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#DAZStage_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#DAZStage_la_LDFLAGS = -module
#
#install:
#	cp .libs/DAZStage.so $(IJPATH)/$(MMPREFIX)DAZStage
#
#EXTRA_DIST = DAZStage.vcproj license.txt
#
