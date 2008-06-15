add_subdirectory(MMCore)
add_subdirectory(MMDevice)
add_subdirectory(MMCoreJ_wrap)
add_subdirectory(bin)
add_subdirectory(mmstudio)
add_subdirectory(DeviceAdapters)
add_subdirectory(Bleach)
add_subdirectory(autofocus)
add_subdirectory(ModuleTest)
add_subdirectory(Test_MMCore)
add_subdirectory(Test_Serial)
add_subdirectory(DeviceAdapters)


########### install files ###############




#original Makefile.am contents follow:

#AUTOMAKE_OPTIONS = foreign
#
#SUBDIRS =  MMCore MMDevice MMCoreJ_wrap bin mmstudio DeviceAdapters Bleach autofocus ModuleTest Test_MMCore Test_Serial DeviceAdapters 
#
#EXTRA_DIST = mmbuild.bat lib/empty.txt build_instructions_win.html build_instructions_unix.txt
#
