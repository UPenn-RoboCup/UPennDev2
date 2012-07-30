CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )

FIND_PACKAGE(lua REQUIRED)

set(LibName OccMap)
set(LibSrc 
  KmeanCluster.cpp
  OccMap.cpp
  luaOccMap.cc
)

add_library(${LibName} MODULE ${LibSrc} )

set_target_properties(${LibName} PROPERTIES PREFIX "")
target_link_libraries(${LibName} ${COMM_LIBS})
include_directories(${LUA_INCLUDE_DIR})
FILE(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/*.lua")
install (
  FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
  PERMISSIONS ${FILE_PERMISSIONS}
)

install(TARGETS ${LibName}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
  PERMISSIONS ${FILE_PERMISSIONS}
)
