# Makefile used to build Webots files
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: unix stty getch

unix: luaunix.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

stty: luastty.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

getch: getch.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

serial: luaserial.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

clean:
	rm -f *.o *.$(SHLIBEXT)
