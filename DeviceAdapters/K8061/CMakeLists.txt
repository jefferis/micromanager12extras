
########### next target ###############

SET(K8061_LIB_SRCS
   K8061.cpp
   K8061.h
   K8061Interface.h
   K8061Interface.cpp
   ../../MMDevice/MMDevice.h
   ../../MMDevice/DeviceBase.h
)

add_library(K8061 SHARED ${K8061_LIB_SRCS})

target_link_libraries(K8061 MMDevice)

set_target_properties(K8061 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS K8061 DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = K8061.la
#K8061_la_SOURCES = K8061.cpp K8061.h K8061Interface.h K8061Interface.cpp \
#   ../../MMDevice/MMDevice.h ../../MMDevice/DeviceBase.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#K8061_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#K8061_la_LDFLAGS = -module
#
#install:
#	cp .libs/K8061.so $(IJPATH)/$(MMPREFIX)K8061
