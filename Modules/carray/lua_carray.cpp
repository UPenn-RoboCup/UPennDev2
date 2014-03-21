/*
	 Lua module to provide efficient access to C arrays

	 Daniel D. Lee <ddlee@seas.upenn.edu>
	 Stephen McGill <smcgill3@seas.upenn.edu>
	 Yida Zhang <yida@seas.upenn.edu>
	 */

#include <lua.hpp>
#include <string.h>

#define MT_NAME "carray_mt"
#define LUA_TCDATA 10

typedef unsigned char byte;
typedef unsigned int uint;

typedef struct {
	void *ptr;
	char type;
	int size;
	int own; // 1 if array was created by Lua and needs to be deleted
} structCArray;

static structCArray * lua_checkcarray(lua_State *L, int narg) {
	void *ud = luaL_checkudata(L, narg, MT_NAME);
	luaL_argcheck(L, *(structCArray **)ud != NULL, narg, "invalid carray");
	return (structCArray *)ud;
}

static int lua_carray_null(lua_State *L) {
	lua_pushlightuserdata(L, 0);
	return 1;
}

// Template function to create new <char>, <int>, ... carray objects
template<typename T, char name>
static int lua_carray_new(lua_State *L) {
	structCArray *ud = (structCArray *)lua_newuserdata(L, sizeof(structCArray));
	ud->type = name;
	// Cannot make variables inside switch/case
	size_t len;
	const char *str;
	T *ptr;
	//
	switch(lua_type(L, 1)){
		case LUA_TCDATA:
		case LUA_TLIGHTUSERDATA:
			ud->size = luaL_optint(L, 2, 1);  // Get optional size argument
			ud->own = 0; // Do not free memory when deleting
			ud->ptr = (void *) lua_topointer(L, 1);
			break;
		case LUA_TNUMBER:
			ud->size = lua_tointeger(L, 1);
			ud->own = 1;
			ud->ptr = new T[ud->size];
			memset(ud->ptr, 0, sizeof(T)*ud->size);
			break;
		case LUA_TSTRING:
			str = lua_tolstring(L, 1, &len);
			ud->size = len/sizeof(T);
			ud->own = 1;
			ud->ptr = new char[len];
			memcpy(ud->ptr, str, len);
			break;
		case LUA_TTABLE:
#if LUA_VERSION_NUM == 502
			ud->size = lua_rawlen(L, 1);
#else
			ud->size = lua_objlen(L, 1);
#endif
			ud->own = 1;
			ptr = new T[ud->size];
			for (int i = 0; i < ud->size; i++) {
				lua_pushinteger(L, i+1);
				lua_gettable(L, 1);
				ptr[i] = lua_tonumber(L, -1);
				lua_pop(L, 1); // previous value
			}
			ud->ptr = ptr;
			break;
		default:
			return luaL_error(L, "Unknown initializer");
	}
	luaL_getmetatable(L, MT_NAME);
	lua_setmetatable(L, -2);
	return 1;
}

static int lua_carray_delete(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	if (p->own) {
		switch (p->type) {
			case 'b':
				delete (unsigned char *)p->ptr;
				break;
			case 'c':
				delete (char *)p->ptr;
				break;
			case 's':
				delete (short *)p->ptr;
				break;
			case 'l':
				delete (long *)p->ptr;
				break;
			case 'i':
				delete (int *)p->ptr;
				break;
			case 'u':
				delete (unsigned int *)p->ptr;
				break;
			case 'f':
				delete (float *)p->ptr;
				break;
			case 'd':
				delete (double *)p->ptr;
				break;
			default:
				delete (char *)p->ptr;
		}
	}
	return 0;
}

static int lua_carray_setValue(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	int index = luaL_checkint(L, 2) - 1; // Convert lua 1-index to C 0-index
	if ((index < 0) || (index >= p->size)) {
		return luaL_error(L, "index out of bounds");
	}

	double val = lua_tonumber(L, 3);
	switch (p->type) {
		case 'b':
			((unsigned char *)p->ptr)[index] = val;
			break;
		case 'c':
			((char *)p->ptr)[index] = val;
			break;
		case 's':
			((short *)p->ptr)[index] = val;
			break;
		case 'l':
			((long *)p->ptr)[index] = val;
			break;
		case 'i':
			((int *)p->ptr)[index] = val;
			break;
		case 'u':
			((unsigned int *)p->ptr)[index] = val;
			break;
		case 'f':
			((float *)p->ptr)[index] = val;
			break;
		case 'd':
			((double *)p->ptr)[index] = val;
			break;
		default:
			((char *)p->ptr)[index] = val;
	}

	return 0;
}

static int lua_carray_getValue(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	int index = luaL_checkint(L, 2) - 1; // Convert lua 1-index to C 0-index

	if ((index < 0) || (index >= p->size)) {
		lua_pushnil(L);
		return 1;
	}

	double val;
	switch (p->type) {
		case 'b':
			val = ((unsigned char *)p->ptr)[index];
			break;
		case 'c':
			val = ((char *)p->ptr)[index];
			break;
		case 's':
			val = ((short *)p->ptr)[index];
			break;
		case 'l':
			val = ((long *)p->ptr)[index];
			break;
		case 'i':
			val = ((int *)p->ptr)[index];
			break;
		case 'u':
			val = ((unsigned int *)p->ptr)[index];
			break;
		case 'f':
			val = ((float *)p->ptr)[index];
			break;
		case 'd':
			val = ((double *)p->ptr)[index];
			break;
		default:
			lua_pushnil(L);
			return 1;
	}
	lua_pushnumber(L, val);
	return 1;
}

static int lua_carray_index(lua_State *L) {
	if(lua_checkcarray(L, 1) && (lua_type(L, 2) == LUA_TNUMBER) && lua_tointeger(L, 2)) {
		// Numeric index:
		return lua_carray_getValue(L);
	}

	// Get index through metatable:
	if (!lua_getmetatable(L, 1)) {
		lua_pop(L, 1);
		return 0;
	}
	// push metatable
	lua_pushvalue(L, 2); // copy key
	lua_rawget(L, -2); // get metatable function
	lua_remove(L, -2); // delete metatable
	return 1;
}

static int lua_carray_pointer(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	//  int offset = luaL_optint(L, 2, 0);

	size_t len;
	switch (p->type) {
		case 'b':
			len = p->size*sizeof(unsigned char);
			break;
		case 'c':
			len = p->size*sizeof(char);
			break;
		case 's':
			len = p->size*sizeof(short);
			break;
		case 'l':
			len = p->size*sizeof(long);
			break;
		case 'i':
			len = p->size*sizeof(int);
			break;
		case 'u':
			len = p->size*sizeof(unsigned int);
			break;
		case 'f':
			len = p->size*sizeof(float);
			break;
		case 'd':
			len = p->size*sizeof(double);
			break;
		default:
			len = p->size;
	}

	lua_pushlightuserdata(L, p->ptr);

	lua_pushinteger(L, len);

	//  lua_pushlightuserdata(L, ((char *)p->ptr + offset));
	return 2;
}

static int lua_carray_typename(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	lua_pushfstring(L, "%c", p->type);
	return 1;
}


// Copy carray to Lua table
static int lua_carray_totable(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	int start = luaL_optint(L, 2, 1)-1;
	int stop = luaL_optint(L, 3,  p->size);
	int tbl_idx = 1;

	lua_createtable(L, stop-start+1, 0);
	for (int i = start; i < stop; i++) {
		double val;
		switch (p->type) {
			case 'b':
				val = ((unsigned char *)p->ptr)[i];
				break;
			case 'c':
				val = ((char *)p->ptr)[i];
				break;
			case 's':
				val = ((short *)p->ptr)[i];
				break;
			case 'l':
				val = ((long *)p->ptr)[i];
				break;
			case 'i':
				val = ((int *)p->ptr)[i];
				break;
			case 'u':
				val = ((unsigned int *)p->ptr)[i];
				break;
			case 'f':
				val = ((float *)p->ptr)[i];
				break;
			case 'd':
				val = ((double *)p->ptr)[i];
				break;
			default:
				val = 0;
		}
		lua_pushnumber(L, val);
		lua_rawseti(L, -2, tbl_idx++);
	}
	return 1;
}

// Copy carray to char string
static int lua_carray_tostring(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	/*
		 lua_pushfstring(L, "carray(%p): '%c' type, %d len, %d own",
		 p->ptr, p->type, p->size, p->own);
		 */
	size_t len;
	switch (p->type) {
		case 'b':
			len = p->size*sizeof(unsigned char);
			break;
		case 'c':
			len = p->size*sizeof(char);
			break;
		case 's':
			len = p->size*sizeof(short);
			break;
		case 'l':
			len = p->size*sizeof(long);
			break;
		case 'i':
			len = p->size*sizeof(int);
			break;
		case 'u':
			len = p->size*sizeof(unsigned int);
			break;
		case 'f':
			len = p->size*sizeof(float);
			break;
		case 'd':
			len = p->size*sizeof(double);
			break;
		default:
			len = p->size;
	}

	lua_pushlstring(L, (const char *)p->ptr, len);
	return 1;
}

static int lua_carray_len(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	lua_pushinteger(L, p->size);
	return 1;
}

static int lua_carray_bytesize(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);
	switch (p->type) {
		case 'b':
			lua_pushinteger(L, p->size);
			break;
		case 'c':
			lua_pushinteger(L, p->size);
			break;
		case 's':
			lua_pushinteger(L, p->size*sizeof(short));
			break;
		case 'l':
			lua_pushinteger(L, p->size*sizeof(long));
			break;
		case 'i':
			lua_pushinteger(L, p->size*sizeof(int));
			break;
		case 'u':
			lua_pushinteger(L, p->size*sizeof(unsigned int));
			break;
		case 'f':
			lua_pushinteger(L, p->size*sizeof(float));
			break;
		case 'd':
			lua_pushinteger(L, p->size*sizeof(double));
			break;
		default:
			lua_pushnil(L);
	}
	return 1;
}



static int lua_carray_cast(lua_State *L) {
	/*
	// Check for lightuserdata carray
	if (!lua_islightuserdata(L, 1)) {
	lua_pushnil(L);
	return 1;
	}
	*/
	const char *type = luaL_optstring(L, 2, "double");
	int size = luaL_optinteger(L, 3, 1);
	structCArray *ud = (structCArray *)lua_newuserdata(L, sizeof(structCArray));
	ud->ptr = (void *)lua_topointer(L, 1);
	ud->size = size;
	ud->type = type[0];
	ud->own = 0;

	luaL_getmetatable(L, "carray_mt");
	lua_setmetatable(L, -2);
	return 1;
}

static int lua_carray_fpointer(lua_State *L) {
	structCArray *p = lua_checkcarray(L, 1);

	lua_pushlightuserdata(L, (void *) p->ptr);
	lua_pushlstring(L, &(p->type), 1);
	lua_pushinteger(L, p->size);
	return 3;
}

static int lua_carray_equality(lua_State *L) {
	structCArray *p1 = lua_checkcarray(L, 1);
	structCArray *p2 = lua_checkcarray(L, 2);
	void * ptr1 = p1->ptr;
	void * ptr2 = p2->ptr;

	if (p1->type != p2->type) {
		lua_pushboolean(L, 0);
		return 1;
	}
	if (p1->size != p2->size) {
		lua_pushboolean(L, 0);
		return 1;
	}
	for (int i = 0; i < p1->size; i++) {
		switch (p1->type) {
			case 'b':
				if (((unsigned char *)ptr1)[i] != ((unsigned char *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			case 'c':
				if (((char *)ptr1)[i] != ((char *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			case 's':
				if (((short *)ptr1)[i] != ((short *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			case 'l':
				if (((long *)ptr1)[i] != ((long *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			case 'i':
				if (((int *)ptr1)[i] != ((int *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			case 'u':
				if (((unsigned int *)ptr1)[i] != ((unsigned int *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			case 'f':
				if (((float *)ptr1)[i] != ((float *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			case 'd':
				if (((double *)ptr1)[i] != ((double *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
				break;
			default:
				if (((char *)ptr1)[i] != ((char *)ptr2)[i]) {
					lua_pushboolean(L, 0);
					return 1;
				}
		}
	}

	lua_pushboolean(L, 1);
	return 1;
}

static const luaL_Reg carray_functions[] = {
	{"null", lua_carray_null},
	{"byte", lua_carray_new<byte, 'b'>},
	{"char", lua_carray_new<char, 'c'>},
	{"short", lua_carray_new<short, 's'>},
	{"long", lua_carray_new<long, 'l'>},
	{"uint", lua_carray_new<uint, 'u'>},
	{"int", lua_carray_new<int, 'i'>},
	{"float", lua_carray_new<float, 'f'>},
	{"double", lua_carray_new<double, 'd'>},
	{"cast", lua_carray_cast},
	{"pointer", lua_carray_fpointer},
	{NULL, NULL}
};

static const luaL_Reg carray_methods[] = {
	{"pointer", lua_carray_pointer},
	{"bytesize", lua_carray_bytesize},
	{"typename", lua_carray_typename},
	{"table", lua_carray_totable},
	{"string", lua_carray_tostring},
	{"__gc", lua_carray_delete},
	{"__newindex", lua_carray_setValue},
	{"__tostring", lua_carray_tostring},
	{"__len", lua_carray_len},
	{"__index", lua_carray_index},
	{"__eq", lua_carray_equality},
	{NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_carray (lua_State *L) {
	// Make the metatable for this carray
	luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
	// TODO: why 0 for nup? Any use for nup?
	luaL_setfuncs( L, carray_methods, 0 );
	luaL_newlib( L, carray_functions );
#else
	luaL_register(L, NULL, carray_methods);
	luaL_register(L, "carray", carray_functions);
#endif

	return 1;
}
