cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

FIND_PACKAGE(lua REQUIRED)

set(ZLIB_SRC zlib-src)
set(LibName Z)
set(LibSrc 
  luaZ.cpp 
  ${ZLIB_SRC}/deflate.c 
  ${ZLIB_SRC}/inflate.c 
  ${ZLIB_SRC}/inffast.c 
  ${ZLIB_SRC}/inftrees.c 
  ${ZLIB_SRC}/trees.c
  ${ZLIB_SRC}/adler32.c 
  ${ZLIB_SRC}/crc32.c
  ${ZLIB_SRC}/zutil.c
)



INCLUDE_DIRECTORIES(${WEBOTS_INCLUDE_C_DIR})
add_library(${LibName} MODULE ${LibSrc} )
include_directories(${LUA_INCLUDE_DIR})
set_target_properties(${LibName} PROPERTIES PREFIX "")
target_link_libraries(${LibName} ${LUA_LIBRARY})

FILE(GLOB files 
  "${CMAKE_CURRENT_SOURCE_DIR}/*.lua")
install (
  FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
)

install(TARGETS ${LibName}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
)
