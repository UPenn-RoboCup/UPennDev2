
CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )
MARK_AS_ADVANCED( CMAKE_BACKARDS_COMPATIBILITY )

# Define Project
PROJECT( UPennalizer_Library )
MESSAGE( STATUS "Configuring Project ${PROJECT_NAME}" )

# Find lua
FIND_PACKAGE(Lua51 REQUIRED)

# Find boost
FIND_PACKAGE(boost REQUIRED)

OPTION( NAO "Compile Libraries and Configure for Nao" OFF )
OPTION( OP "Compile Libraries and Configure for OP" OFF )
OPTION( WEBOTS "Compile Libraries and Configure for Webots Controller" OFF )

set( LIB_SRC "${PROJECT_SOURCE_DIR}/Lib" )
set( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/Player/Lib" )
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/Lib/Modules" )

INCLUDE_DIRECTORIES( ${LUA_INCLUDE_DIR} ${BOOST_INCLUDE_DIR} )
set( COMMLIBS LUA)

IF ( APPLE )
	MESSAGE( STATUS "Configure for APPLE" )
ENDIF( APPLE )

SUBDIRS( 
	${LIB_SRC}/ImageProc
	${LIB_SRC}/Util
)

