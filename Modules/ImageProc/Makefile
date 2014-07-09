LIBNAME=ImageProc
EXTRA_OBJ=color_count.o block_bitor.o \
	ConnectRegions.o RegionProps.o RadonTransform.o\
	lua_color_stats.o lua_color_count.o lua_colorlut_gen.o \
	lua_connect_regions.o \
	lua_goal_posts.o lua_robots.o\
	lua_field_lines.o lua_field_spots.o lua_field_occupancy.o \
	lua_fast_12.o fast_12.o \
	lua_obstacles.o
include ../../Makefile.inc
LDFLAGS+=-lm
ifdef USE_TORCH
	CXXFLAGS+=-DTORCH=1
	LDFLAGS+=-ltorch
endif
