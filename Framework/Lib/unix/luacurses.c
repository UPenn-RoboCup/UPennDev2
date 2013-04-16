/*
  Lua interface to the ncurses library 
*/

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#include <ncurses.h>

typedef const struct const_info {
  const char *name;
  int value;
} const_info;

void lua_install_constants(lua_State *L, const_info constants[]) {
  int i;
  for (i = 0; constants[i].name; i++) {
    lua_pushstring(L, constants[i].name);
    lua_pushinteger(L, constants[i].value);
    lua_rawset(L, -3);
  }
}

/* Setup */

static int luacurses_initscr(lua_State *L) {
  initscr();
  lua_pushboolean(L, 1);
  return 1;
}

static int luacurses_endwin(lua_State *L) {
  lua_pushboolean(L, endwin() != ERR);
  return 1;
}

/* Options */

static int luacurses_timeout(lua_State *L) {
  int delay = luaL_checkint(L, 1);
  timeout(delay);
  lua_pushboolean(L, 1);
  return 1;
}

static int luacurses_raw(lua_State *L) {
  lua_pushboolean(L, raw() != ERR);
  return 1;
}

static int luacurses_noraw(lua_State *L) {
  lua_pushboolean(L, noraw() != ERR);
  return 1;
}

static int luacurses_echo(lua_State *L) {
  lua_pushboolean(L, echo() != ERR);
  return 1;
}

static int luacurses_noecho(lua_State *L) {
  lua_pushboolean(L, noecho() != ERR);
  return 1;
}

static int luacurses_cbreak(lua_State *L) {
  lua_pushboolean(L, cbreak() != ERR);
  return 1;
}

static int luacurses_nocbreak(lua_State *L) {
  lua_pushboolean(L, nocbreak() != ERR);
  return 1;
}

static int luacurses_keypad(lua_State *L) {
  int enable = luaL_checkint(L, 1);
  lua_pushboolean(L, keypad(stdscr, enable) != ERR);
  return 1;
}

/* Motion */

static int luacurses_move(lua_State *L) {
  int y = luaL_checkint(L, 1);
  int x = luaL_checkint(L, 2);
  lua_pushboolean(L, move(y, x) != ERR);
  return 1;
}

static int luacurses_getyx(lua_State *L) {
 int y, x; 
 getyx(stdscr, y, x);
 lua_pushinteger(L, y);
 lua_pushinteger(L, x);
 return 2;
}

/* I/O */

static int luacurses_refresh(lua_State *L) {
  lua_pushboolean(L, refresh() != ERR);
  return 1;
}

static int luacurses_clear(lua_State *L) {
  lua_pushboolean(L, clear() != ERR); 
  return 1;
}

static int luacurses_addch(lua_State *L) {
  int ch;
  ch = luaL_checkint(L, 1);   
  lua_pushboolean(L, addch(ch) != ERR);
  return 1;
}

static int luacurses_addstr(lua_State *L) {
  const char *str = luaL_checkstring(L, 1);
  lua_pushboolean(L, addstr(str) != ERR);
  return 1;
}

static int luacurses_addnstr(lua_State *L) {
  const char *str = luaL_checkstring(L, 1);
  int n = luaL_checkint(L, 2);
  lua_pushboolean(L, addnstr(str, n) != ERR);
  return 1;
}

static int luacurses_printw(lua_State *L) {
  lua_getglobal(L, "string");
  lua_getfield(L, -1, "format");
  lua_insert(L, 1);
  lua_pop(L, 1);
  lua_pcall(L, lua_gettop(L)-1, 1, 0);
  const char *str = luaL_checkstring(L, 1);
  lua_pushboolean(L, addstr(str) != ERR);
  return 1;
}

static int luacurses_getch(lua_State *L) {
  int ch = getch();
  if (ch == ERR) 
    lua_pushnil(L);
  else
    lua_pushinteger(L, ch);
  return 1;
}

static int luacurses_ungetch(lua_State *L) {
  int ch = luaL_checkint(L, 1);
  lua_pushboolean(L, ungetch(ch) != ERR);
  return 1;
}

static int luacurses_getstr(lua_State *L) {
  char str[256];
  if (getstr(str) == ERR)
    lua_pushnil(L);
  else
    lua_pushstring(L, str);
  return 1;
}

static int luacurses_getnstr(lua_State *L) {
  char str[256];
  int n = luaL_checkint(L, 1);
  if (getnstr(str, n) == ERR)
    lua_pushnil(L);
  else
    lua_pushstring(L, str);
  return 1;
}

static int luacurses_scanw(lua_State *L) {
  char str[256];
  if (getstr(str) == ERR) {
    lua_pushboolean(L, 0);
    return 1;
  }
  lua_pushstring(L, str);
  lua_insert(L, 1);
  lua_getglobal(L, "string");
  lua_getfield(L, -1, "match");
  lua_insert(L, 1);
  lua_pop(L, 1);
  lua_settop(L, 3);
  lua_pcall(L, 2, LUA_MULTRET, 0);
  return lua_gettop(L);
}

static int luacurses_resizeterm(lua_State *L) {
  int lines = luaL_checkint(L, 1);
  int columns = luaL_checkint(L, 2);
  lua_pushinteger(L, resizeterm(lines, columns));
  return 1;
}

static int luacurses_getmaxyx(lua_State *L) {
  int y;
  int x;
  getmaxyx(stdscr, y, x);
  lua_pushinteger(L, y);
  lua_pushinteger(L, x);
  return 2;
}

/* Attributes */

static int luacurses_attron(lua_State *L) {
  int A = luaL_checkint(L, 1);
  lua_pushboolean(L, attron(A) != ERR);
  return 1;
}

static int luacurses_attroff(lua_State *L) {
  int A = luaL_checkint(L, 1);
  lua_pushboolean(L, attroff(A) != ERR);
  return 1;
}

static const struct luaL_Reg curses_lib [] = {
  {"initscr", luacurses_initscr},
  {"endwin", luacurses_endwin},
  {"timeout", luacurses_timeout},
  {"raw", luacurses_raw},
  {"noraw", luacurses_noraw},
  {"echo", luacurses_echo},
  {"noecho", luacurses_noecho},
  {"cbreak", luacurses_cbreak},
  {"nocbreak", luacurses_nocbreak},
  {"keypad", luacurses_keypad},
  {"move", luacurses_move},
  {"getyx", luacurses_getyx},
  {"clear", luacurses_clear},
  {"refresh", luacurses_refresh},
  {"addch", luacurses_addch},
  {"addstr", luacurses_addstr},
  {"addnstr", luacurses_addnstr},
  {"printw", luacurses_printw},
  {"getch", luacurses_getch},
  {"ungetch", luacurses_ungetch},
  {"getstr", luacurses_getstr},
  {"getnstr", luacurses_getnstr},
  {"attron", luacurses_attron},
  {"attroff", luacurses_attroff},
  {"scanw", luacurses_scanw},
  {"resizeterm", luacurses_resizeterm},
  {"getmaxyx", luacurses_getmaxyx},
  {NULL, NULL}
};

static const const_info curses_constants[] = {
  {"A_ALTCHARSET", A_ALTCHARSET},
  {"A_BLINK", A_BLINK},
  {"A_BOLD", A_BOLD},
  {"A_DIM", A_DIM},
  {"A_NORMAL", A_NORMAL},
  {"A_REVERSE", A_REVERSE},
  {"A_STANDOUT", A_STANDOUT},
  {"A_UNDERLINE", A_UNDERLINE},
  {"KEY_MIN", KEY_MIN},
  {"KEY_BREAK", KEY_BREAK},
  {"KEY_DOWN", KEY_DOWN},
  {"KEY_UP", KEY_UP},
  {"KEY_LEFT", KEY_LEFT},
  {"KEY_RIGHT", KEY_RIGHT},
  {"KEY_HOME", KEY_HOME},
  {"KEY_BACKSPACE", KEY_BACKSPACE},
  {"KEY_F0", KEY_F0},
  {"KEY_DL", KEY_DL},
  {"KEY_IL", KEY_IL},
  {"KEY_DC", KEY_DC},
  {"KEY_IC", KEY_IC},
  {"KEY_EIC", KEY_EIC},
  {"KEY_CLEAR", KEY_CLEAR},
  {"KEY_EOS", KEY_EOS},
  {"KEY_EOL", KEY_EOL},
  {"KEY_SF", KEY_SF},
  {"KEY_SR", KEY_SR},
  {"KEY_NPAGE", KEY_NPAGE},
  {"KEY_PPAGE", KEY_PPAGE},
  {"KEY_STAB", KEY_STAB},
  {"KEY_CTAB", KEY_CTAB},
  {"KEY_CATAB", KEY_CATAB},
  {"KEY_ENTER", KEY_ENTER},
  {"KEY_SRESET", KEY_SRESET},
  {"KEY_RESET", KEY_RESET},
  {"KEY_PRINT", KEY_PRINT},
  {"KEY_LL", KEY_LL},
  {"KEY_A1", KEY_A1},
  {"KEY_A3", KEY_A3},
  {"KEY_B2", KEY_B2},
  {"KEY_C1", KEY_C1},
  {"KEY_C3", KEY_C3},
  {"KEY_BTAB", KEY_BTAB},
  {"KEY_BEG", KEY_BEG},
  {"KEY_CANCEL", KEY_CANCEL},
  {"KEY_CLOSE", KEY_CLOSE},
  {"KEY_COMMAND", KEY_COMMAND},
  {"KEY_COPY", KEY_COPY},
  {"KEY_CREATE", KEY_CREATE},
  {"KEY_END", KEY_END},
  {"KEY_EXIT", KEY_EXIT},
  {"KEY_FIND", KEY_FIND},
  {"KEY_HELP", KEY_HELP},
  {"KEY_MARK", KEY_MARK},
  {"KEY_MESSAGE", KEY_MESSAGE},
  {"KEY_MOVE", KEY_MOVE},
  {"KEY_NEXT", KEY_NEXT},
  {"KEY_OPEN", KEY_OPEN},
  {"KEY_OPTIONS", KEY_OPTIONS},
  {"KEY_PREVIOUS", KEY_PREVIOUS},
  {"KEY_REDO", KEY_REDO},
  {"KEY_REFERENCE", KEY_REFERENCE},
  {"KEY_REFRESH", KEY_REFRESH},
  {"KEY_REPLACE", KEY_REPLACE},
  {"KEY_RESTART", KEY_RESTART},
  {"KEY_RESUME", KEY_RESUME},
  {"KEY_SAVE", KEY_SAVE},
  {"KEY_SBEG", KEY_SBEG},
  {"KEY_SCANCEL", KEY_SCANCEL},
  {"KEY_SCOMMAND", KEY_SCOMMAND},
  {"KEY_SCOPY", KEY_SCOPY},
  {"KEY_SCREATE", KEY_SCREATE},
  {"KEY_SDC", KEY_SDC},
  {"KEY_SDL", KEY_SDL},
  {"KEY_SELECT", KEY_SELECT},
  {"KEY_SEND", KEY_SEND},
  {"KEY_SEOL", KEY_SEOL},
  {"KEY_SEXIT", KEY_SEXIT},
  {"KEY_SFIND", KEY_SFIND},
  {"KEY_SHELP", KEY_SHELP},
  {"KEY_SHOME", KEY_SHOME},
  {"KEY_SIC", KEY_SIC},
  {"KEY_SLEFT", KEY_SLEFT},
  {"KEY_SMESSAGE", KEY_SMESSAGE},
  {"KEY_SMOVE", KEY_SMOVE},
  {"KEY_SNEXT", KEY_SNEXT},
  {"KEY_SOPTIONS", KEY_SOPTIONS},
  {"KEY_SPREVIOUS", KEY_SPREVIOUS},
  {"KEY_SPRINT", KEY_SPRINT},
  {"KEY_SREDO", KEY_SREDO},
  {"KEY_SREPLACE", KEY_SREPLACE},
  {"KEY_SRIGHT", KEY_SRIGHT},
  {"KEY_SRSUME", KEY_SRSUME},
  {"KEY_SSAVE", KEY_SSAVE},
  {"KEY_SSUSPEND", KEY_SSUSPEND},
  {"KEY_SUNDO", KEY_SUNDO},
  {"KEY_SUSPEND", KEY_SUSPEND},
  {"KEY_UNDO", KEY_UNDO},
  {"KEY_MOUSE", KEY_MOUSE},
  {"KEY_RESIZE", KEY_RESIZE},
  {"KEY_MAX", KEY_MAX},
  {NULL, 0}
};

int luaopen_curses (lua_State *L) {
#if LUA_VERSION_NUM == 502
	  luaL_newlib(L, curses_lib);
#else
  luaL_register(L, "curses", curses_lib);
#endif
	lua_install_constants(L, curses_constants);
  return 1;
}
