include ../../../Makefile.inc

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

all: unix stty getch curses

unix: luaunix.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

stty: luastty.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

getch: luagetch.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

curses: luacurses.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIB_DIRS) -lncurses $^

clean:
	rm -f *.o *.$(SHLIBEXT)