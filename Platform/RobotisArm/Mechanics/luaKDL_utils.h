#ifndef _LUA_KDL_TYPES_H
#define _LUA_KDL_TYPES_H

extern "C"
{
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

inline void lua_createvector(lua_State *L)
{
  lua_getglobal(L, "vector");
  if (lua_istable(L, -1))
  {
    lua_getfield(L, -1, "new");
    lua_call(L, 0, 1);
  }
  else
  {
    lua_createtable(L, 0, 0);
  }
  lua_remove(L, -2);
}

inline void lua_createtransform(lua_State *L)
{
  lua_getglobal(L, "Transform");
  if (lua_istable(L, -1))
  {
    lua_getfield(L, -1, "eye");
    lua_call(L, 0, 1);
  }
  else
  {
    lua_createtable(L, 4, 0);
    for (int i = 0; i < 4; i++)
    {
      lua_createtable(L, 4, 0);
      lua_rawseti(L, -2, i+1);
    }
  }
  lua_remove(L, -2);
}

inline void lua_pushJntArray(lua_State *L, KDL::JntArray q)
{
  int n = q.rows();
  lua_createvector(L);
  for (int i = 0; i < n; i++)
  {
    lua_pushnumber(L, q(i));
    lua_rawseti(L, -2, i+1);
  }
}

inline void lua_pushVector(lua_State *L, KDL::Vector v)
{
  lua_createvector(L);
  for (int i = 0; i < 3; i++)
  {
    lua_pushnumber(L, v(i));
    lua_rawseti(L, -2, i+1);
  }
}

inline void lua_pushFrame(lua_State *L, KDL::Frame f)
{
  lua_createtransform(L);
  for (int i = 0; i < 4; i++)
  {
    lua_rawgeti(L, -1, i+1); 
    for (int j = 0; j < 4; j++)
    {
      lua_pushnumber(L, f(i, j));
      lua_rawseti(L, -2, j+1);
    }
    lua_pop(L, 1);
  }
}

inline void lua_pushTwist(lua_State *L, KDL::Twist t)
{
  lua_createvector(L);
  for (int i = 0; i < 6; i++)
  {
    lua_pushnumber(L, t(i));
    lua_rawseti(L, -2, i+1);
  }
}

inline void lua_pushWrench(lua_State *L, KDL::Wrench w)
{
  lua_createvector(L);
  for (int i = 0; i < 6; i++)
  {
    lua_pushnumber(L, w(i));
    lua_rawseti(L, -2, i+1);
  }
}

inline void lua_pushJacobian(lua_State *L, KDL::Jacobian J)
{
  int nr = J.rows();
  int nc = J.columns(); 
  lua_createtable(L, nr, 0);
  for (int i = 0; i < nr; i++)
  {
    lua_createtable(L, nc, 0);
    for (int j = 0; j < nc; j++)
    {
      lua_pushnumber(L, J(i, j));
      lua_rawseti(L, -2, j+1);
    }
    lua_rawseti(L, -2, i+1);
  }
}

inline KDL::JntArray lua_checkJntArray(lua_State *L, int index)
{
  luaL_checktype(L, index, LUA_TTABLE);
  int n = lua_objlen(L, index);
  KDL::JntArray q(n);

  for (int i = 0; i < n; i++)
  {
    lua_rawgeti(L, index, i+1);
    q(i) = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
  return q;
}

inline KDL::Vector lua_checkVector(lua_State *L, int index)
{
  luaL_checktype(L, index, LUA_TTABLE);
  if (lua_objlen(L, index) != 3)
    luaL_argerror(L, index, "invalid vector length");
  KDL::Vector v;

  for (int i = 0; i < 3; i++)
  {
    lua_rawgeti(L, index, i+1);
    v(i) = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
  return v;
}

inline KDL::Frame lua_checkFrame(lua_State *L, int index)
{
  luaL_checktype(L, index, LUA_TTABLE);
  if (lua_objlen(L, index) != 4)
    luaL_argerror(L, index, "invalid transform dimensions");
  KDL::Vector v;
  KDL::Rotation M;

  for (int i = 0; i < 3; i++)
  {
    lua_rawgeti(L, index, i+1);
    for (int j = 0; j < 3; j++)
    {
      lua_rawgeti(L, -1, j+1);
      M(i, j) = lua_tonumber(L, -1);
      lua_pop(L, 1);
    }
    lua_rawgeti(L, -1, 4);
    v(i) = lua_tonumber(L, -1);
    lua_pop(L, 2);
  }
  return KDL::Frame(M, v);
}

inline KDL::Twist lua_checkTwist(lua_State *L, int index)
{
  luaL_checktype(L, index, LUA_TTABLE);
  if (lua_objlen(L, index) != 6)
    luaL_argerror(L, index, "invalid twist length");
  KDL::Twist t;

  for (int i = 0; i < 6; i++)
  {
    lua_rawgeti(L, index, i+1);
    t(i) = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
  return t;
}

inline KDL::Wrench lua_checkWrench(lua_State *L, int index)
{
  luaL_checktype(L, index, LUA_TTABLE);
  if (lua_objlen(L, index) != 6)
    luaL_argerror(L, index, "invalid wrench length");
  KDL::Wrench w;

  for (int i = 0; i < 6; i++)
  {
    lua_rawgeti(L, index, i+1);
    w(i) = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
  return w;
}

#endif
