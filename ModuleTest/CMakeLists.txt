
########### next target ###############

SET(moduletest_SRCS
   ModuleTest.cpp
)

add_executable(moduletest ${moduletest_SRCS})

target_link_libraries(moduletest MMCore)

install(TARGETS moduletest DESTINATION bin)


########### install files ###############




#original Makefile.am contents follow:

#bin_PROGRAMS = moduletest
#moduletest_SOURCES = ModuleTest.cpp 
#moduletest_LDADD = ../MMCore/libMMCore.a $(LIBACE)
###moduletest_SOURCES = ModuleTest.cpp ../MMCore/PluginManager.cpp
###moduletest_LDADD = /usr/local/lib/libACE.la
#
#EXTRA_DIST = ModuleTest.vcproj ModuleTest.sln
