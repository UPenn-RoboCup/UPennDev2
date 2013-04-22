include ../../../Makefile.inc

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

all: unix stty getch curses

%: lua%.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^
	rm lua$@.o

curses: luacurses.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(LIBOPTS) $(LIB_DIRS) -lncurses

clean:
	rm -f *.o *.$(SHLIBEXT)
