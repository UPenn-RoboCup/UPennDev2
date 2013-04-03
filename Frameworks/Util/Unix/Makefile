# Makefile used to build Webots files
include ../../../Makefile.inc

all: unix stty getch

unix: luaunix.o
	@echo "\t" LINK $@
	@$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

stty: luastty.o
	@echo "\t" LINK $@
	@$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

getch: getch.o
	@echo "\t" LINK $@
	@$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

serial: luaserial.o
	@echo "\t" LINK $@
	@$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.o *.$(SHLIBEXT)
