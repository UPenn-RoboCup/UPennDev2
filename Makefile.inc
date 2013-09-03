# Makefile Includes

OSTYPE=$(shell uname -s|awk '{print tolower($$0)}')
	
# From Julia Make.inc
ENDCOLOR="\033[0m"
SRCCOLOR="\033[33m"
CCCOLOR="\033[34m"
CXXCOLOR="\033[32;1m"
LINKCOLOR="\033[34;1m"
BINCOLOR="\033[37;1m"
INFOCOLOR="\033[32m"
CLEANCOLOR="\033[35m"

## Compiler Definition
# Defaults to Linux
CXX=c++
CC=cc
LD=g++
INCLUDES=
LIBOPTS=
LDFLAGS=-shared -fpic
STRIP=strip
CXXFLAGS=-O3 -fpic
CFLAGS=-O3 -fpic
#CFLAGS+= -mtune=native -march=native
#CXXFLAGS+= -mtune=native -march=native
LIBRT=-lrt
SHLIBEXT=so

USE_TORCH=1

# OS Specific overrides
ifeq ($(OSTYPE),darwin)
  LIBRT=
  LD=ld -macosx_version_min 10.7
  LDFLAGS=-lstdc++ -bundle -undefined dynamic_lookup
endif

# Add Lua flags
LUA_VERSION=5.1
# Find LuaJIT first
ifeq ($(shell pkg-config --exists luajit && echo 0),0)
	LUAJIT_INC=`pkg-config luajit --cflags`
	LUAJIT_LIB=`pkg-config luajit --libs`
	#USE_LUAJIT=yes
endif
# Find Pure Lua
ifeq ($(shell pkg-config --exists lua$(LUA_VERSION) && echo 0),0)
	LUA_INC=`pkg-config lua$(LUA_VERSION) --cflags`
	LUA_LIB=`pkg-config lua$(LUA_VERSION) --libs`
else
	LUA_INC=`pkg-config lua --cflags`
	LUA_LIB=`pkg-config lua --libs`
endif

ifdef USE_LUAJIT
	INCLUDES+=$(LUAJIT_INC)
	LIBOPTS+=$(LUAJIT_LIB)
else
	INCLUDES+=$(LUA_INC)
	LIBOPTS+=$(LUA_LIB)
endif

# Verbose mode check
V=@
ifdef VERBOSE
V=
endif

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
	$(V)$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
$(LIBNAME): lua_$(LIBNAME).o $(EXTRA_OBJ)
	$(V)printf '\t%b %b\n' $(LINKCOLOR)LINK$(ENDCOLOR) $(BINCOLOR)$@$(ENDCOLOR);
	$(V)$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(EXTRA_LINK) $(LIBOPTS)
	$(V)rm $^
clean:
	$(V)printf '\t%b %b\n' $(CLEANCOLOR)Cleaning$(ENDCOLOR) $(shell pwd);
	$(V)rm -f *.so *.o $(EXTRA_CLEAN)
