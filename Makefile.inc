# Makefile Includes
OSTYPE=$(shell uname -s|awk '{print tolower($$0)}')

# Compiler Definitions
CXX=c++
CC=cc
CXXFLAGS=-O3 -fPIC -std=c++03 -pedantic -Wall
CFLAGS=-O3 -fPIC -std=c99 -pedantic -Wall
LIBRT=-lrt

# Linker Definitions
LD=g++
LDFLAGS=-shared
SHLIBEXT=so
STRIP=strip
USE_TORCH=1
INCLUDES=-I/usr/local/include/

# OS Specific overrides
ifeq ($(OSTYPE),darwin)
  LIBRT=
  LD=ld -macosx_version_min 10.9
  LDFLAGS=-lstdc++ -bundle -undefined dynamic_lookup
else # Linux
  CFLAGS+= -mtune=native -march=native `pkg-config lua5.1 --cflags`
  CXXFLAGS+= -mtune=native -march=native `pkg-config lua5.1 --cflags`
endif

# Verbose mode check
V=@
ifdef VERBOSE
V=
endif

# From Julia Make.inc
ENDCOLOR="\033[0m"
SRCCOLOR="\033[33m"
CCCOLOR="\033[34m"
CXXCOLOR="\033[32;1m"
LINKCOLOR="\033[34;1m"
BINCOLOR="\033[37;1m"
INFOCOLOR="\033[32m"
CLEANCOLOR="\033[35m"

# Default compiling for modules
all none: $(LIBNAME)
%.o: %.cpp
	$(V)printf '\t%b %b\n' $(CXXCOLOR)CXX$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(V)printf '\t%b %b\n' $(CXXCOLOR)CXX$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(V)printf '\t%b %b\n' $(CCCOLOR)CC$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<
$(LIBNAME): lua_$(LIBNAME).o $(EXTRA_OBJ)
	$(V)printf '\t%b %b\n' $(LINKCOLOR)LINK$(ENDCOLOR) $(BINCOLOR)$@$(ENDCOLOR);
	$(V)$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(EXTRA_LINK) $(LIBOPTS)
	$(V)rm $^
clean:
	$(V)printf '\t%b %b\n' $(CLEANCOLOR)Cleaning$(ENDCOLOR) $(shell pwd);
	$(V)rm -f *.so *.o $(EXTRA_CLEAN)
