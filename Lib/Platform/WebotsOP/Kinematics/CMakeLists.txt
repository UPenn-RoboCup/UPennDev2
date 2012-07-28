CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )

FIND_PACKAGE(lua REQUIRED)

set(LibName OPKinematics)
set(LibSrc 
    OPKinematics.cpp 
    Transform.cpp 
    luaOPKinematics.cpp
)

add_library(${LibName} MODULE ${LibSrc} )

set_target_properties(${LibName} PROPERTIES PREFIX "")
target_link_libraries(${LibName} ${LUA_LIBRARY})

FILE(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/*.lua")
install (
  FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
)

install(TARGETS ${LibName}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
)
