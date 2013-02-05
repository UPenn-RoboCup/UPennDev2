# Makefile used to build libraries
include ../../../Lib/Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

QTINC=-I/usr/include/qt4 -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtCore

all: q

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(QTINC) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

q: luaQ.o  lqt_common.o lqt_qt.o
	$(LD) $(LDFLAGS) -o Q.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -lQtCore -lQtGui

simplepng: simplepng.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

clean:
	rm -f *.$(SHLIBEXT) *.o
