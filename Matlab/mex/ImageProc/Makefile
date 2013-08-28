# Makefile used to build mex files
TARGETS = all clean
.PHONY: $(TARGETS)
MEXEXT=$(shell mexext)
MEX=mex -O

all: yuyv2label.$(MEXEXT) \
	block_bitor.$(MEXEXT) \
	color_count.$(MEXEXT) \
	color_stats.$(MEXEXT) \
	connected_regions.$(MEXEXT) \
	goal_posts.$(MEXEXT) \
	field_lines.$(MEXEXT) \
	field_occupancy.$(MEXEXT) \
	field_spots.$(MEXEXT)\
	label_to_mask.$(MEXEXT)

%.o: %.cc
	$(CXX) -fpic -O3 -o $@ -c $<

%.$(MEXEXT): %.cc
	$(MEX) $<

connected_regions.$(MEXEXT): connected_regions.cc RegionProps.o ConnectRegions.o
	$(MEX) $^

goal_posts.$(MEXEXT): goal_posts.cc RegionProps.o
	$(MEX) $^

goal_crossbar.$(MEXEXT): goal_crossbar.cc RegionProps.o
	$(MEX) $^

field_lines.$(MEXEXT): field_lines.cc RadonTransform.o
	$(MEX) $^

field_spots.$(MEXEXT): field_spots.cc RegionProps.o ConnectRegions.o
	$(MEX) $^

clean:
	rm -f *.$(MEXEXT) *.o
