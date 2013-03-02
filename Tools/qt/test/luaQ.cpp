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

#include <iostream>
#include "lqt_common.hpp"
#include "QtGui"
#include "lqt_qt.hpp"


using namespace std;

static int lua_q_init(lua_State *L) {
  if (lua_type(L, 1)!=10)
    luaL_typerror(L, 1, "cdata");
  unsigned char const * buffer = (unsigned char const*) lua_topointer(L, 1);
  int w = luaL_checkint(L, 2);
  int h = luaL_checkint(L, 3);

  for (int i = 0; i < w * h; i+=3) {
    cout << (unsigned int)buffer[i] << ' ';
    cout << (unsigned int)buffer[i + 1] << ' ';
    cout << (unsigned int)buffer[i + 2] << endl;
  }
  cout << w << ' ' << h << endl;
  return 1;
}

static int lua_q_show(lua_State *L) {
  QImage* self = static_cast<QImage*>(lqtL_toudata(L, 1, "QImage*"));
  lqtL_selfcheck(L, self, "QImage");
  unsigned char* data = self->bits();
  cout << (unsigned short)*data << endl;
  int w = luaL_checkint(L, 2);
  int h = luaL_checkint(L, 3);

//  for (int i = 0; i < w * h; i+=3) {
//    cout << (unsigned int)data[i] << ' ';
//    cout << (unsigned int)data[i + 1] << ' ';
//    cout << (unsigned int)data[i + 2] << endl;
//  }
  cout << w << ' ' << h << endl;
  return 1;
}

static int lua_q_udata(lua_State *L) {
  QVector<QRgb> const arg1 = *static_cast<QVector<QRgb>*>(lqtL_toudata(L, 1, "QVector<QRgb>*"));
//  self->QImage::setColorTable(arg1);
  return 1;
}

static const struct luaL_reg Q_lib [] = {
  {"init", lua_q_init},
  {"show", lua_q_show},
  {"udata", lua_q_udata},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_Q (lua_State *L) {
  luaL_register(L, "Q", Q_lib);

  return 1;
}

