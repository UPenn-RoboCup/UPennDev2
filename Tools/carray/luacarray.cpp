/*
  Lua module to provide efficient access to C arrays
*/

#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif
  #include "lua.h"
  #include "lualib.h"
  #include "lauxlib.h"
#ifdef __cplusplus
}
#endif

#define MT_NAME "carray_mt"
#define LUA_TCDATA 10

#include <iostream>

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

// Template function to create new <char>, <int>, ... carray object
template<typename T, char name>
static int lua_carray_new(lua_State *L) {
  structCArray *ud = (structCArray *)lua_newuserdata(L, sizeof(structCArray));
  ud->type = name;

  if (lua_type(L, 1) == LUA_TCDATA) {
//    ud->size = luaL_optint(L, -2, 1);  // Get optional size argument
    ud->size = lua_tointeger(L, 2);  // Get optional size argument
    ud->own = 0; // Do not free memory when deleting
    ud->ptr = (void *) lua_topointer(L, 1);
  }
  else if (lua_type(L, 1) == LUA_TLIGHTUSERDATA) { // Cast from pointer
    ud->size = luaL_optint(L, -2, 1);  // Get optional size argument
    ud->own = 0; // Do not free memory when deleting
    ud->ptr = (void *) lua_topointer(L, 1);
  }
  else if (lua_type(L, 1) == LUA_TNUMBER) { // Allocate array with size
    ud->size = lua_tointeger(L, 1);
    ud->own = 1;
    ud->ptr = new T[ud->size];
    memset(ud->ptr, 0, sizeof(T)*ud->size);
  }
  else if (lua_type(L, 1) == LUA_TSTRING) { // Copy from string
    size_t len;
    const char *str = lua_tolstring(L, 1, &len);

    ud->size = len/sizeof(T);
    ud->own = 1;
    char *p = new char[len];
    ud->ptr = p;
    memcpy(p, str, len);
  }
  else if (lua_type(L, 1) == LUA_TTABLE) { // Initialize with Lua table
    ud->size = lua_objlen(L, 1);
    ud->own = 1;
    T *p = new T[ud->size];
    ud->ptr = p;
    for (int i = 0; i < ud->size; i++) {
      lua_pushinteger(L, i+1);
      lua_gettable(L, 1);
      p[i] = lua_tonumber(L, -1);
      lua_pop(L, 1); // previous value
    }
  }
  else { // Allocate array with size
    ud->size = 1;
    ud->own = 1;
    ud->ptr = new T[ud->size];
    memset(ud->ptr, 0, sizeof(T)*ud->size);
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
  structCArray *p = lua_checkcarray(L, 1);
  if ((lua_type(L, 2) == LUA_TNUMBER) && lua_tointeger(L, 2)) {
    // Numeric index:
    return lua_carray_getValue(L);
  }

  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}

static int lua_carray_pointer(lua_State *L) {
  structCArray *p = lua_checkcarray(L, 1);
  int offset = luaL_optint(L, 2, 0);

  lua_pushlightuserdata(L, ((char *)p->ptr + offset));
  return 1;
}

static int lua_carray_typename(lua_State *L) {
  structCArray *p = lua_checkcarray(L, 1);
  lua_pushfstring(L, "%c", p->type);
  return 1;
}

// Copy carray to Lua table
static int lua_carray_totable(lua_State *L) {
  structCArray *p = lua_checkcarray(L, 1);
  lua_createtable(L, p->size, 0);
  for (int i = 0; i < p->size; i++) {
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
    lua_rawseti(L, -2, i+1);
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

static const struct luaL_reg carray_functions[] = {
  {"null", lua_carray_null},
  {"byte", lua_carray_new<byte, 'b'>},
  {"char", lua_carray_new<char, 'c'>},
  {"short", lua_carray_new<short, 's'>},
  {"long", lua_carray_new<long, 'l'>},
  {"uint", lua_carray_new<uint, 'u'>},
  {"int", lua_carray_new<int, 'i'>},
  {"float", lua_carray_new<float, 'f'>},
  {"double", lua_carray_new<double, 'd'>},
  {NULL, NULL}
};

static const struct luaL_reg carray_methods[] = {
  {"pointer", lua_carray_pointer},
  {"typename", lua_carray_typename},
  {"table", lua_carray_totable},
  {"string", lua_carray_tostring},
  {"__gc", lua_carray_delete},
  {"__newindex", lua_carray_setValue},
  {"__tostring", lua_carray_tostring},
  {"__len", lua_carray_len},

  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_carray (lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

  // Implement index method:
  lua_pushstring(L, "__index");
  lua_pushcfunction(L, lua_carray_index);
  lua_settable(L, -3);

  luaL_register(L, NULL, carray_methods);
  luaL_register(L, "carray", carray_functions);

  return 1;
}
