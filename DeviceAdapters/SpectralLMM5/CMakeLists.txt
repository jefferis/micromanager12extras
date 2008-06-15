
########### next target ###############

SET(SpectralLMM5_LIB_SRCS
   SpectralLMM5.cpp
   SpectralLMM5.h
   SpectralLMM5Interface.cpp
   SpectralLMM5Interface.h
)

add_library(SpectralLMM5 SHARED ${SpectralLMM5_LIB_SRCS})

target_link_libraries(SpectralLMM5 MMDevice)

set_target_properties(SpectralLMM5 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS SpectralLMM5 DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = SpectralLMM5.la
#SpectralLMM5_la_SOURCES = SpectralLMM5.cpp SpectralLMM5.h SpectralLMM5Interface.cpp SpectralLMM5Interface.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#SpectralLMM5_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#SpectralLMM5_la_LDFLAGS = -module
#
#install:
#	cp .libs/SpectralLMM5.so $(IJPATH)/$(MMPREFIX)SpectralLMM5
#
#EXTRA_DIST =  license.txt
