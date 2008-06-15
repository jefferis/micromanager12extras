
########### next target ###############

SET(CSU22_LIB_SRCS
   CSU22.cpp
   CSU22.h
   CSU22Hub.h
   CSU22Hub.cpp
   ../../MMDevice/MMDevice.h
   ../../MMDevice/DeviceBase.h
   ../../MMDevice/ModuleInterface.h
)

add_library(CSU22 SHARED ${CSU22_LIB_SRCS})

target_link_libraries(CSU22 MMDevice)

set_target_properties(CSU22 PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS CSU22 DESTINATION lib)


########### next target ###############

SET(CSUX_LIB_SRCS
   CSUX.cpp
   CSUX.h
   CSUXHub.h
   CSUXHub.cpp
   ../../MMDevice/MMdevice.h
   ../../MMDevice/DeviceBase.h
   ../../MMDevice/ModuleInterface.h
)

add_library(CSUX SHARED ${CSUX_LIB_SRCS})

target_link_libraries(CSUX MMDevice)

set_target_properties(CSUX PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS CSUX DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive 
#lib_LTLIBRARIES = CSU22.la CSUX.la
#CSU22_la_SOURCES = CSU22.cpp CSU22.h CSU22Hub.h CSU22Hub.cpp ../../MMDevice/MMDevice.h ../../MMDevice/DeviceBase.h ../../MMDevice/ModuleInterface.h
#CSU22_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
#CSU22_la_LDFLAGS = -module
#
#CSUX_la_SOURCES = CSUX.cpp CSUX.h CSUXHub.h CSUXHub.cpp ../../MMDevice/MMdevice.h ../../MMDevice/DeviceBase.h ../../MMDevice/ModuleInterface.h
#CSUX_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a
#CSUX_la_LDFLAGS = -module
#
#install:
#	cp .libs/CSU22.so $(IJPATH)/$(MMPREFIX)CSU22
#	cp .libs/CSUX.so $(IJPATH)/$(MMPREFIX)CSUX
