cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.5.3")
project("SolARModuleTools")
set (SOURCES 
		src/SolARImage2WorldMapper4Marker2D.cpp
		src/SolAR2DTransform.cpp
		src/SolAR3DTransform.cpp
		src/SolARHomographyValidation.cpp
		src/SolARSBPatternReIndexer.cpp
		src/SolARKeypointsReIndexer.cpp
		src/SolARBasicMatchesFilter.cpp
		src/SolARMapper.cpp
		src/SolARMapFilter.cpp
		src/SolARModuleTools.cpp
		src/SolARKeyframeSelector.cpp
		src/SolARBasicSink.cpp
		src/SolARBasicSource.cpp
)

set (HEADERS
		interfaces/SolARImage2WorldMapper4Marker2D.h
		interfaces/SolAR2DTransform.h
		interfaces/SolAR3DTransform.h
		interfaces/SolARHomographyValidation.h
		interfaces/SolARSBPatternReIndexer.h
		interfaces/SolARKeypointsReIndexer.h
		interfaces/SolARMapper.h
		interfaces/SolARMapFilter.h
		interfaces/SolARModuleManagerTools.h
		interfaces/SolARToolsAPI.h
		interfaces/SolARModuleTools_traits.h
		interfaces/SolARBasicMatchesFilter.h
		interfaces/SolARKeyframeSelector.h
		interfaces/SolARBasicSink.h
		interfaces/SolARBasicSource.h
)

install (FILES "${CMAKE_CURRENT_SOURCE_DIR}/xpcf_SolARModuleTools_registry.xml" DESTINATION $ENV{BCOMDEVROOT}/.xpcf/SolAR/	)

##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library" "")


