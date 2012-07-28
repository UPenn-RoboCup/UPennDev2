set(CMAKE_CXX_COMPILER "/usr/bin/g++")

IF ( APPLE )
	MESSAGE( STATUS "Configure for APPLE" )
  set(CMAKE_CXX_FLAGS "-O2 -fomit-frame-pointer -mtune=native" CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS ${CMAKE_CXX_FLAGS} CACHE STRING "" FORCE)
ENDIF( APPLE )

IF ( UNIX )
	MESSAGE( STATUS "Configure for UNIX" )
ENDIF( UNIX )


