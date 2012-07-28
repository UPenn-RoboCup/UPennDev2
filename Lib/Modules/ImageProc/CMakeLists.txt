cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

FIND_PACKAGE(lua REQUIRED)

set(LibName ImageProc)
set(LibSrc 
    block_bitor.cpp
    color_count.cpp
    ConnectRegions.cpp
    lua_robots.cpp
    lua_color_stats.cpp
    lua_color_count.cpp
    lua_colorlut_gen.cpp
    lua_connect_regions.cpp
    lua_field_lines.cpp
    lua_field_occupancy.cpp
    lua_field_spots.cpp
    lua_goal_posts.cpp
    lua_ImageProc.cpp
    RadonTransform.cpp
    RegionProps.cpp
)



INCLUDE_DIRECTORIES(${WEBOTS_INCLUDE_C_DIR})
add_library(${LibName} MODULE ${LibSrc} )
include_directories(${LUA_INCLUDE_DIR})
set_target_properties(${LibName} PROPERTIES PREFIX "")
target_link_libraries(${LibName} ${COMM_LIBS})

FILE(GLOB files 
  "${CMAKE_CURRENT_SOURCE_DIR}/*.lua")
install (
  FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
  PERMISSIONS ${FILE_PERMISSIONS}
)

install(TARGETS ${LibName}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/Lib
  PERMISSIONS ${FILE_PERMISSIONS}
)
