
########### next target ###############

SET(SerialManager_LIB_SRCS
   SerialManager.cpp
   SerialManager.h
   SerialPort.cpp
   SerialPort.h
)

add_library(SerialManager SHARED ${SerialManager_LIB_SRCS})

target_link_libraries(SerialManager MMDevice)

set_target_properties(SerialManager PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS SerialManager DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = SerialManager.la
#SerialManager_la_SOURCES = SerialManager.cpp SerialManager.h \
#         SerialPort.cpp SerialPort.h
### adding the libtool library gives problems on OS X 10.4, adding the static lib works
#SerialManager_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#SerialManager_la_LDFLAGS = -module $(SERIALFRAMEWORKS)
#
#install:
#	cp .libs/SerialManager.so $(IJPATH)/$(MMPREFIX)SerialManager
#
#EXTRA_DIST = license.txt
