
########### next target ###############

SET(Conix_LIB_SRCS
   Conix.cpp
   Conix.h
)

add_library(Conix SHARED ${Conix_LIB_SRCS})

target_link_libraries(Conix MMDevice)

set_target_properties(Conix PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS Conix DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = Conix.la
#Conix_la_SOURCES = Conix.cpp Conix.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#Conix_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
#Conix_la_LDFLAGS = -module
#
#install:
#	cp .libs/Conix.so $(IJPATH)/$(MMPREFIX)Conix
