
########### next target ###############

SET(testserial_SRCS
   Test_Serial.cpp
)

add_executable(testserial ${testserial_SRCS})

target_link_libraries(testserial MMCore MMDevice)

install(TARGETS testserial DESTINATION bin)


########### install files ###############




#original Makefile.am contents follow:

### Process this file with automake to produce Makefile.in
#bin_PROGRAMS = testserial
#testserial_SOURCES = Test_Serial.cpp 
#testserial_LDADD = ../MMCore/libMMCore.a ../MMDevice/.libs/libMMDevice.a $(LIBACE)
#
#EXTRA_DIST = Test_Serial.vcproj Test_Serial.sln stdafx.cpp stdafx.h
