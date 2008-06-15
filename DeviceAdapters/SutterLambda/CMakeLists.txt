
########### next target ###############

SET(SutterLambda_LIB_SRCS
   SutterLambda.cpp
   SutterLambda.h
)

add_library(SutterLambda SHARED ${SutterLambda_LIB_SRCS})

target_link_libraries(SutterLambda MMDevice)

set_target_properties(SutterLambda PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS SutterLambda DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = SutterLambda.la
#SutterLambda_la_SOURCES = SutterLambda.cpp SutterLambda.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#SutterLambda_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#SutterLambda_la_LDFLAGS = -module
#
#install:
#	cp .libs/SutterLambda.so $(IJPATH)/$(MMPREFIX)SutterLambda
#
#EXTRA_DIST = SutterLambda.vcproj license.txt
