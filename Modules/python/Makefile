LIBNAME=python
EXTRA_OBJ=luainpython.o
include ../../Makefile.inc
INCLUDES+=`python-config --includes`
LDFLAGS+=`python-config --libs`
#LUA_LIBS=`pkg-config luajit --libs`
LUA_LIBS=`pkg-config lua --libs`

lua: luainpython.o lua_python.o
	@printf '\t%b %b\n' $(LINKCOLOR)LINK$(ENDCOLOR) $(BINCOLOR)$@$(ENDCOLOR);
	$(V)$(LD) -o $@.$(SHLIBEXT) $^ $(LDFLAGS) $(LUA_LIBS)

# Make both the module for python and for lua
all none: $(LIBNAME) lua
