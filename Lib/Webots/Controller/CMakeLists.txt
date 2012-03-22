set(SYMLINK_DIR "${CMAKE_INSTALL_PREFIX}/${WEBOTSCONTROLLER_INST}")
#message("${SYMLINK_DIR}")

set(SYMLINKS
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/nao_team_0"
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/nao_team_1"
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/darwin-op_team_0"
	"${SYMLINK_DIR}/lua_set.sh->${SYMLINK_DIR}/darwin-op_team_1"
	"${PLAYER_DIR}->${SYMLINK_DIR}/Player"
)

configure_file(
	"${CMAKE_MODULE_PATH}/create_symlink.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/create_symlink.cmake"
	IMMEDIATE @ONLY
)


ADD_LIBRARY( controller SHARED controller_wrap.c )
INCLUDE_DIRECTORIES( ${WEBOTS_INCLUDE_C_DIR} )
TARGET_LINK_LIBRARIES( controller ${COMMLIBS} ${WEBOTS_LIBRARY_C} )
SET_TARGET_PROPERTIES( controller PROPERTIES PREFIX "" )
#INSTALL_TARGETS( ${WEBOTSCONTROLLER_INST} controller )
#INSTALL_FILES( ${WEBOTSCONTROLLER_INST} lua_set.sh )
#INSTALL_FILES( ${WEBOTSCONTROLLER_INST} start.lua )

INSTALL( SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/create_symlink.cmake )
