# Makefile used to build libraries
include ../../../Lib/Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

QTINC=`pkg-config QtGui --cflags` `pkg-config QtCore --cflags`

all: q

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(QTINC) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

q: luaQ.o  lqt_common.o lqt_qt.o
	$(LD) $(LDFLAGS) -o Q.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm `pkg-config QtGui --libs` `pkg-config QtCore --libs`

simplepng: simplepng.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

clean:
	rm -f *.$(SHLIBEXT) *.o
