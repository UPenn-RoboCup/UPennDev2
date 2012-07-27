
CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )
MARK_AS_ADVANCED( CMAKE_BACKARDS_COMPATIBILITY )

IF ( COMMAND cmake_policy )
	cmake_policy( SET CMP0003 NEW )
ENDIF( COMMAND cmake_policy )



# Define Project
PROJECT( "UPennalizer Library" )
MESSAGE( STATUS "Configuring Project ${PROJECT_NAME}" )

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
endif (setup_webots_nao)

if (setup_webots_op)
  message( "-- Compile and Configure Libraries for WebotsOP")
  option(WebotsOP on)
  option(Webots on)
endif (setup_webots_op)

## uninstall target
#configure_file(
#    "${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in"
#    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
#    IMMEDIATE @ONLY)
#
#add_custom_target(uninstall
#    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
#
#configure_file(
#	"${CMAKE_MODULE_PATH}/create_symlink.cmake.in"
#	"${CMAKE_CURRENT_BINARY_DIR}/create_symlink.cmake"
#)
#
#OPTION( NAO "Compile Libraries and Configure for Nao" OFF )
#OPTION( OP "Compile Libraries and Configure for OP" OFF )
#OPTION( XOS "Compile Libraries and Configure for XOS" OFF )
#OPTION( WEBOTS "Compile Libraries and Configure for Webots Controller" OFF )
#
#
#set( LIB_SRC "${PROJECT_SOURCE_DIR}/Lib" )
#set( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}" )
#set( LIBRARY_INST "/Player/Lib" )
#set( WEBOTSCONTROLLER_INST "WebotsController" )
#set( PLAYER_DIR "${PROJECT_SOURCE_DIR}/Player" )
#INCLUDE_DIRECTORIES( ${LUA_INCLUDE_DIR} ${BOOST_INCLUDE_DIR} )
#set( COMMLIBS ${LUA_LIBRARY})
#
## Set make clean
#SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_INSTALL_PREFIX}/Lib")
#
#IF ( APPLE )
#	MESSAGE( STATUS "Configure for APPLE" )
#ENDIF( APPLE )
#
#IF ( UNIX )
#	MESSAGE( STATUS "Configure for UNIX" )
#ENDIF( UNIX )
#
#SUBDIRS( 
#	${LIB_SRC}/ImageProc
#	${LIB_SRC}/Util
#	${LIB_SRC}/Platform
#	${LIB_SRC}/Webots
#	${LIB_SRC}/Comm
#)

