cmake_minimum_required(VERSION 2.8)

##################################################
set (VERSION_NUMBER "0.3.1")
project("SolARModuleTools")
set (SOURCES 
		src/SolARImage2WorldMapper4Marker2D.cpp
		src/SolAR2DTransform.cpp
		src/SolAR3DTransform.cpp
		src/SolARHomographyValidation.cpp
		src/SolARSBPatternReIndexer.cpp
		src/SolARKeypointsReIndexer.cpp
		src/SolARModuleTools.cpp
		src/SolARModuleManagerTools.cpp
	)
##################################################

# various macros
include("macros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (library, install and uninstall)
defineTargets()


