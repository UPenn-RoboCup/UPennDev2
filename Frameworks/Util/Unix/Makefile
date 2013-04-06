# Makefile used to build Webots files
include ../../../Makefile.inc

all: unix stty getch signal

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

signal: lua_signal.o
	@echo "\t" LINK $@
	@$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.o *.$(SHLIBEXT)
