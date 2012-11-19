MEX_EXT = $(shell mexext)

TARGETS = ScanMatch2D.$(MEX_EXT) getMapCellsFromRay.$(MEX_EXT) binStats.$(MEX_EXT)

all: $(TARGETS)

%.$(MEX_EXT): %.cc
	mex -O $^

%.$(MEX_EXT): %.cpp
	mex -O $^


clean:
	rm -rf $(TARGETS) *~
	
