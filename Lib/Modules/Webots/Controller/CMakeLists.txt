CMAKE_MINIMUM_REQUIRED ( VERSION 2.4.6 )

FIND_PACKAGE(Webots REQUIRED)

set(LibName controller)
set(LibSrc 
    controller_wrap.c
)

INCLUDE_DIRECTORIES(${WEBOTS_INCLUDE_C_DIR})
add_library(${LibName} MODULE ${LibSrc} )
include_directories(${LUA_INCLUDE_DIR})

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

set(symdir ${CMAKE_INSTALL_PREFIX}/../WebotsController)
set(symlinks 
  "lua_set.sh->darwin-op_team_1"
  "lua_set.sh->darwin-op_team_2"
  "lua_set.sh->nao_team_1"
  "lua_set.sh->nao_team_2")

foreach(symlink ${symlinks})
	string(REGEX REPLACE "->" ";" SRC_DEST "${symlink}")
	list(GET SRC_DEST 0 SRC)
  list(GET SRC_DEST 1 DEST) 
  #	message("${SRC} ${DEST}")
  install(CODE "execute_process(COMMAND @CMAKE_COMMAND@ -E create_symlink ${SRC} ${symdir}/${DEST} OUTPUT_VARIABLE ln_out	RESULT_VARIABLE ln_retvel)"
	)
  install(CODE "
	if (NOT ${ln_retval} EQUAL 0)
		message(FATAL_ERROR \"Problem when creating symbolic link from ${SRC} to ${DEST}\")
	else (NOT ${ln_retval} EQUAL 0)
    message(STATUS \"Creating symbolic link from ${SRC} to ${DEST}\")
	endif (NOT ${ln_retval} EQUAL 0)
  ")
endforeach(symlink ${symlinks})
