
########### next target ###############

SET(QCam_LIB_SRCS
   QICamera.cpp
   QICamera.h
)

add_library(QCam SHARED ${QCam_LIB_SRCS})

target_link_libraries(QCam MMDevice)

set_target_properties(QCam PROPERTIES VERSION 4.2.0 SOVERSION 4)
install(TARGETS QCam DESTINATION lib)


########### install files ###############




#original Makefile.am contents follow:

#AM_CXXFLAGS = -fpermissive
#lib_LTLIBRARIES = QCam.la
#QCam_la_SOURCES = QICamera.cpp QICamera.h
### adding the libtool library gives problems on Intel OS X 10.4, adding the static lib works
#QCam_la_LIBADD = ../../MMDevice/.libs/libMMDevice.a 
#QCam_la_LDFLAGS = -module -framework QCam
#
#install:
#	cp .libs/QCam.so $(IJPATH)/$(MMPREFIX)QCam
#
#EXTRA_DIST = QCam.vcproj
