cmake_minimum_required(VERSION 2.6)

add_subdirectory(AOTF)
add_subdirectory(Andor)
add_subdirectory(ASIFW1000)
add_subdirectory(ASIStage)
add_subdirectory(DAZStage)
add_subdirectory(DTOpenLayer)
add_subdirectory(DemoCamera)
add_subdirectory(DemoStreamingCamera)
add_subdirectory(SerialManager)
add_subdirectory(SerialManagerUNIX)
add_subdirectory(Ludl)
add_subdirectory(LudlLow)
add_subdirectory(PI)
add_subdirectory(PI_GCS)
add_subdirectory(Prior)
add_subdirectory(ParallelPort)
add_subdirectory(Pecon)
add_subdirectory(Sensicam)
add_subdirectory(ZeissCAN)
add_subdirectory(ZeissCAN29)
add_subdirectory(Vincent)
add_subdirectory(Conix)
add_subdirectory(Nikon)
add_subdirectory(StanfordPhotonics)
add_subdirectory(SpectralLMM5)
add_subdirectory(SutterLambda)
add_subdirectory(Yokogawa)

message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(HAMAMATSU)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(PVCAM)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(QCAM)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(TE2000)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(AZ100)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(K8055)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(K8061)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(USBMANAGER)")
message(STATUS "${CMAKE_CURRENT_SOURCE_DIR}: skipped subdir $(DC1394)")

########### install files ###############




#original Makefile.am contents follow:

#AUTOMAKE_OPTIONS = foreign
#
#if BUILD_HAMAMATSU
#   HAMAMATSU = Hamamatsu
#endif
#if BUILD_PVCAM
#   PVCAM = PVCAM
#endif
#if BUILD_QCAM
#   QCAM = QCam
#endif
###if BUILD_TE2000
###	TE2000 = NikonTE2000
###endif
#if BUILD_AZ100
#	AZ100 = NikonAZ100
#endif
#if BUILD_DC1394
#   DC1394 = dc1394
#endif
#if BUILD_K8055
#   K8055 = K8055
#endif
#if BUILD_K8061
#   K8061 = K8061
#endif
#if BUILD_USBMANAGER
#   USBMANAGER = USBManager 
#endif
#
#SUBDIRS =  AOTF Andor ASIFW1000 ASIStage DAZStage DTOpenLayer DemoCamera DemoStreamingCamera SerialManager SerialManagerUNIX Ludl LudlLow PI PI_GCS Prior ParallelPort Pecon Sensicam ZeissCAN ZeissCAN29 Vincent Conix $(HAMAMATSU) $(PVCAM) $(QCAM) $(TE2000) $(AZ100) $(K8055) $(K8061) $(USBMANAGER) Nikon StanfordPhotonics SpectralLMM5 SutterLambda $(DC1394) Yokogawa
#
