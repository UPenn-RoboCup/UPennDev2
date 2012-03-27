# Makefile used to build libraries
include ../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: ImageProc

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<

ImageProc: luaImageProc.o \
	color_count.o block_bitor.o \
	ConnectRegions.o RegionProps.o RadonTransform.o\
	lua_color_stats.o lua_goal_posts.o \
	lua_field_lines.o lua_field_spots.o lua_field_occupancy.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ -L$(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
