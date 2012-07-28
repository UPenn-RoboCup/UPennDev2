CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )
MARK_AS_ADVANCED( CMAKE_BACKARDS_COMPATIBILITY )

# Define Project
PROJECT( "UPennalizer Library" )
MESSAGE( STATUS "Configuring Project ${PROJECT_NAME}" )

set( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../Player" )
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/Modules/CMake" )

# Find lua
FIND_PACKAGE(lua REQUIRED)

# Find boost
FIND_PACKAGE(boost REQUIRED)


option(NAO "NAO" off)
option(NAOV4 "NAOV4" off)
option(OP "OP" off)
option(XOP "XOP" off)
option(WebotsNao "WebotsNao" off)
option(WebotsOP "WebotsOP" off)
option(Webots "Webots" off)

if (setup_nao)
  message( "-- Compile and Configure Libraries for Nao")
  option(NAO on)
endif (setup_nao)

if (setup_naov4)
  message( "-- Compile and Configure Libraries for NaoV4")
  option(NAOV4 on)
endif (setup_naov4)

if (setup_op)
  message( "-- Compile and Configure Libraries for OP")
  option(OP on)
endif (setup_op)

if (setup_xop)
  message( "-- Compile and Configure Libraries for XOS")
  option(XOP on)
endif (setup_xop)

if (setup_webots_nao)
  message( "-- Compile and Configure Libraries for WebotsNao")
  option(WebotsNao on)
  option(Webots on)
  add_subdirectory("Platform/OP")
  add_subdirectory("Modules/Util")
  add_subdirectory("Modules/Comm")
  add_subdirectory("Modules/ImageProc")
  add_subdirectory("Modules/OccMap")
  set(symlinks "Config_OP.lua->Config.lua")
endif (setup_webots_nao)

if (setup_webots_op)
  message( "-- Compile and Configure Libraries for WebotsOP")
  option(WebotsOP on)
  option(Webots on)
  add_subdirectory("Platform/WebotsOP")
  add_subdirectory("Modules/Webots")
  add_subdirectory("Modules/Util")
  add_subdirectory("Modules/ImageProc")
  add_subdirectory("Modules/OccMap")
  set(symlinks "Config_WebotsOP.lua->Config.lua")
endif (setup_webots_op)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")

IF ( APPLE )
	MESSAGE( STATUS "Configure for APPLE" )
  set(CMAKE_CXX_FLAGS "-O2 -fomit-frame-pointer -mtune=native" CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING "" FORCE)
ENDIF( APPLE )

IF ( UNIX )
	MESSAGE( STATUS "Configure for UNIX" )
ENDIF( UNIX )


##### Create Symbolic Link for Config #######################
set(symdir ${CMAKE_INSTALL_PREFIX}/Config)
foreach(symlink ${symlinks})
	string(REGEX REPLACE "->" ";" SRC_DEST "${symlink}")
	list(GET SRC_DEST 0 SRC)
  list(GET SRC_DEST 1 DEST) 
  #	message("${SRC} ${DEST}")
  install(CODE "execute_process(COMMAND @CMAKE_COMMAND@ -E create_symlink ${SRC} ${symdir}/${DEST} OUTPUT_VARIABLE ln_out	RESULT_VARIABLE ln_retvel)"
	)
  install(CODE "
	if (NOT ${ln_retval} EQUAL 0)
		message(FATAL_ERROR \"Problem when creating symbolic link from ${SRC} to ${DEST}\")
	else (NOT ${ln_retval} EQUAL 0)
    message(STATUS \"Creating symbolic link from ${SRC} to ${DEST}\")
	endif (NOT ${ln_retval} EQUAL 0)
  ")
endforeach(symlink ${symlinks})
############################################################
