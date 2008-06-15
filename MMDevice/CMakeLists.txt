
########### next target ###############

SET(MMDevice_STAT_SRCS
   ModuleInterface.cpp
   Property.cpp
   DeviceUtils.cpp
   ImgBuffer.cpp
   DeviceBase.h
   MMDevice.h
   MMDeviceConstants.h
   ModuleInterface.h
   Property.h
   DeviceUtils.h
   ImgBuffer.h
   DeviceThreads.h
)

add_library(MMDevice STATIC ${MMDevice_STAT_SRCS})


########### install files ###############




#original Makefile.am contents follow:

#noinst_LTLIBRARIES = libMMDevice.la
#libMMDevice_la_SOURCES = ModuleInterface.cpp Property.cpp DeviceUtils.cpp ImgBuffer.cpp \
#	DeviceBase.h MMDevice.h MMDeviceConstants.h ModuleInterface.h Property.h DeviceUtils.h \
#	ImgBuffer.h DeviceThreads.h
#	
#EXTRA_DIST = license.txt
