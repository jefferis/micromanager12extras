
########### next target ###############

SET(ASIStage_LIB_SRCS
   ASIStage.cpp
   ASIStage.h
#   h
)

add_library(ASIStage SHARED ${ASIStage_LIB_SRCS})

target_link_libraries(ASIStage MMDevice)

set_target_properties(ASIStage PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS ASIStage DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = ASIStage.la
#ASIStage_la_SOURCES = ASIStage.cpp ASIStage.h h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#ASIStage_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#ASIStage_la_LDFLAGS = -module
#
#install:
#	cp .libs/ASIStage.so $(IJPATH)/$(MMPREFIX)ASIStage
#
#EXTRA_DIST = ASIStage.vcproj license.txt
