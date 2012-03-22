
CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )
MARK_AS_ADVANCED( CMAKE_BACKARDS_COMPATIBILITY )

IF ( COMMAND cmake_policy )
	cmake_policy( SET CMP0003 NEW )
ENDIF( COMMAND cmake_policy )



# Define Project
PROJECT( UPennalizer_Library )
MESSAGE( STATUS "Configuring Project ${PROJECT_NAME}" )

set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/Lib/Modules" )

# Find lua
FIND_PACKAGE(lua REQUIRED)

# Find boost
FIND_PACKAGE(boost REQUIRED)

# uninstall target
configure_file(
    "${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

configure_file(
	"${CMAKE_MODULE_PATH}/create_symlink.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/create_symlink.cmake"
)

OPTION( NAO "Compile Libraries and Configure for Nao" OFF )
OPTION( OP "Compile Libraries and Configure for OP" OFF )
OPTION( XOS "Compile Libraries and Configure for XOS" OFF )
OPTION( WEBOTS "Compile Libraries and Configure for Webots Controller" OFF )


set( LIB_SRC "${PROJECT_SOURCE_DIR}/Lib" )
set( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}" )
set( LIBRARY_INST "/Player/Lib" )
set( WEBOTSCONTROLLER_INST "WebotsController" )
set( PLAYER_DIR "${PROJECT_SOURCE_DIR}/Player" )
INCLUDE_DIRECTORIES( ${LUA_INCLUDE_DIR} ${BOOST_INCLUDE_DIR} )
set( COMMLIBS LUA)

# Set make clean
SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_INSTALL_PREFIX}/Lib")

IF ( APPLE )
	MESSAGE( STATUS "Configure for APPLE" )
ENDIF( APPLE )

IF ( UNIX )
	MESSAGE( STATUS "Configure for UNIX" )
ENDIF( UNIX )

SUBDIRS( 
	${LIB_SRC}/ImageProc
	${LIB_SRC}/Util
	${LIB_SRC}/Platform
	${LIB_SRC}/Webots
	${LIB_SRC}/Comm
)

