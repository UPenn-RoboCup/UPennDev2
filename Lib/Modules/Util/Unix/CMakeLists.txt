CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )

FIND_PACKAGE(lua REQUIRED)

set(LibNames unix stty)
set(LibSrc_unix luaunix.c)
set(LibSrc_stty luastty.cpp)
set(LibSrc_serial luaserial.c)
foreach (LibName ${LibNames})
  #  message( "${LibName}")
  add_library(${LibName} MODULE ${LibSrc_${LibName}})
  include_directories(${LUA_INCLUDE_DIR})
  set_target_properties(${LibName} PROPERTIES PREFIX "")
  target_link_libraries(${LibName} ${LUA_LIBRARY})
endforeach (LibName ${LibName})

FILE(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/*.lua")
install (
  FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
)


foreach (LibName ${LibNames})
  install(TARGETS ${LibName}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
  )
endforeach (LibName ${LibName})
