
ADD_LIBRARY(
	ImageProc
	SHARED
	luaImageProc.cpp
	block_bitor.cpp
	color_count.cpp
	ConnectRegions.cpp
	lua_color_stats.cpp
	lua_field_lines.cpp
	lua_field_occupancy.cpp
	lua_field_spots.cpp
	lua_goal_posts.cpp
	RadonTransform.cpp
	RegionProps.cpp	
)

TARGET_LINK_LIBRARIES( ImageProc ${COMMLIBS} )
