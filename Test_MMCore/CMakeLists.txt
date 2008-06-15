
########### next target ###############

SET(testCore_SRCS
   Test_MMCore_Mac.cpp
)

add_executable(testCore ${testCore_SRCS})

target_link_libraries(testCore MMCore MMDevice)

install(TARGETS testCore DESTINATION bin)


########### install files ###############




#original Makefile.am contents follow:

### Process this file with automake to produce Makefile.in
#bin_PROGRAMS = testCore
#testCore_LDADD = ../MMCore/libMMCore.a ../MMDevice/.libs/libMMDevice.a $(LIBACE)
#testCore_SOURCES = Test_MMCore_Mac.cpp 
#
#EXTRA_DIST = Test_MMCore.vcproj Test_MMCore.sln Test_MMCore.cpp
#
#install:
#	cp testCore $(IJPATH)/testCore
#
