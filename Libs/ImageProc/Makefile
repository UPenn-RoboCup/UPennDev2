# Makefile used to build libraries
include ../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: ImageProc

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

ImageProc: lua_ImageProc.o \
	color_count.o block_bitor.o \
	ConnectRegions.o RegionProps.o RadonTransform.o\
	lua_color_stats.o lua_color_count.o lua_colorlut_gen.o \
	lua_connect_regions.o \
	lua_goal_posts.o lua_robots.o\
	lua_field_lines.o lua_field_spots.o lua_field_occupancy.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
