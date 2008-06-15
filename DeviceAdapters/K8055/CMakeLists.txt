
########### next target ###############

SET(K8055_LIB_SRCS
   K8055.cpp
   K8055.h
   K8055Interface.h
   K8055Interface.cpp
)

add_library(K8055 SHARED ${K8055_LIB_SRCS})

target_link_libraries(K8055 MMDevice)

set_target_properties(K8055 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS K8055 DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = K8055.la
#K8055_la_SOURCES = K8055.cpp K8055.h K8055Interface.h K8055Interface.cpp
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#K8055_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#K8055_la_LDFLAGS = -module
#
#install:
#	cp .libs/K8055.so $(IJPATH)/$(MMPREFIX)K8055
