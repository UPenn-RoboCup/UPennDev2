CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )

set(SYMLINK_DIR "${CMAKE_INSTALL_PREFIX}/../WebotsController")

set(SYMLINKS
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/nao_team_0"
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/nao_team_1"
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/darwin-op_team_0"
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/darwin-op_team_1"
	"${PLAYER_DIR}->${SYMLINK_DIR}/Player"
)

configure_file(
  "${CMAKE_MODULE_PATH}/create_symlink.cmake.in"
  IMMEDIATE @ONLY
  )

FIND_PACKAGE(Webots REQUIRED)

set(LibName controller)
set(LibSrc 
    controller_wrap.c
)

INCLUDE_DIRECTORIES(${WEBOTS_INCLUDE_C_DIR})
add_library(${LibName} MODULE ${LibSrc} )

set_target_properties(${LibName} PROPERTIES PREFIX "")
target_link_libraries(${LibName} ${WEBOTS_LIBRARY_C} ${LUA_LIBRARY})


FILE(GLOB files 
  "${CMAKE_CURRENT_SOURCE_DIR}/*.lua"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.sh")
install (
  FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/../WebotsController
)

install(TARGETS ${LibName}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/../WebotsController
)


INSTALL( SCRIPT ${CMAKE_MODULE_PATH}/create_symlink.cmake.in )
