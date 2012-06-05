#include "SoundComm.h"
#include "sound_comm_thread.h"
#include <vector>

// defined in nao_comm_thread.cc
extern bool txPauseCmd;
extern bool rxPauseCmd;
extern bool txPaused;
extern bool rxPaused;

int init = 0;


static int lua_get_sample_size(lua_State *L) {
  lua_pushinteger(L, PFRAME);
  return 1;
}

static int lua_get_sampling_rate(lua_State *L) {
  lua_pushinteger(L, SAMPLING_RATE);
  return 1;
}

static int lua_pause_receiver(lua_State *L) {
  rxPauseCmd = true;
  return 1;
}

static int lua_enable_receiver(lua_State *L) {
  rxPauseCmd = false;
  return 1;
}

static int lua_pause_transmitter(lua_State *L) {
  txPauseCmd = true;
  return 1;
}

static int lua_enable_transmitter(lua_State *L) {
  txPauseCmd = false;
  return 1;
}

static int lua_is_receiver_enabled(lua_State *L) {
  lua_pushboolean(L, !rxPaused);
  return 1;
}

static int lua_is_transmitter_enabled(lua_State *L) {
  lua_pushboolean(L, !txPaused);
  return 1;
}

static int lua_get_detection(lua_State *L) {
  DetStruct det = sound_comm_thread_get_detection();

  // create return struct
  lua_createtable(L, 0, 5);

  lua_pushstring(L, "count");
  lua_pushinteger(L, det.count);
  lua_settable(L, -3);

  lua_pushstring(L, "time");
  lua_pushnumber(L, det.time);
  lua_settable(L, -3);

  lua_pushstring(L, "lIndex");
  lua_pushinteger(L, det.lIndex);
  lua_settable(L, -3);

  lua_pushstring(L, "rIndex");
  lua_pushinteger(L, det.rIndex);
  lua_settable(L, -3);

  lua_pushstring(L, "symbol");
  char s[2];
  s[0] = det.symbol;
  s[1] = '\0';
  lua_pushstring(L, s);
  lua_settable(L, -3);

  return 1;
}


/**
 * function that will queue the audio signal
 *  accepts one or two input signals
 *
 *  if one input then a pcm signal is constructed to play
 *    the signal from both speakers
 *  if two inputs then a pcm signal is constructed to play
 *    the signals out of the left and right speakers respectively
 *    queue_signal(leftAudio, rightAudio)
 */
static int lua_queue_signal(lua_State *L) {
  if (lua_gettop(L) == 2) {
    // check that both inputs are tables
    if (!lua_istable(L, 1) || !lua_istable(L, 2)) {
      fprintf(stderr, "inputs must be arrays.\n");
      return -1;
    }
    // check that both signals are the same size
    if (lua_objlen(L, 1) != lua_objlen(L, 2)) {
      fprintf(stderr, "left and right signals must be the same length.\n");
      return -1;
    }

    // create interleaved pcm signal
    int n = lua_objlen(L, 1);
    std::vector<short> *pcm = new std::vector<short>(n);
    for (int i = 0; i < n; i++) {
      lua_rawgeti(L, 1, i+1);
      (*pcm)[2*i] = (short)lua_tonumber(L, -1);
      lua_rawgeti(L, 2, i+1);
      (*pcm)[2*i+1] = (short)lua_tonumber(L, -1);

      lua_pop(L, 2);
    }

    // queue the pcm signal
    sound_comm_thread_queue_pcm(pcm);

  } else {
    // check that the input is a table
    if (!lua_istable(L, 1)) {
      fprintf(stderr, "input must be an array.\n");
      return -1;
    }

    // create interleaved pcm signal
    int n = lua_objlen(L, 1);
    std::vector<short> *pcm = new std::vector<short>(n);
    for (int i = 0; i < n; i++) {
      lua_rawgeti(L, 1, i+1);
      short t = (short)lua_tonumber(L, -1);
      (*pcm)[2*i] = t;
      (*pcm)[2*i+1] = t;
      lua_pop(L, 1);
    }

    // queue the pcm signal
    sound_comm_thread_queue_pcm(pcm);
  }

  return 1;
}


/**
 * function that will queue the audio PCM
 *
 * the input array is expected to already be in the correct format
 *  - interleaved
 *  - 2 channel
 */
static int lua_queue_pcm(lua_State *L) {
  // check input argument is a table
  if (!lua_istable(L, 1)) {
    fprintf(stderr, "queue_pcm: input must be a table array.\n");
    return -1;
  }
  
  // create vector array from lua table array
  int n = lua_objlen(L, 1);
  std::vector<short> *pcm = new std::vector<short>(n);
  for (int i = 0; i < n; i++) {
    lua_rawgeti(L, 1, i+1);
    (*pcm)[i] = (short)lua_tonumber(L, -1);
    lua_pop(L, 1);
  }

  // queue the pcm signal
  sound_comm_thread_queue_pcm(pcm);

  return 1;
}


static const struct luaL_reg sound_comm_lib [] = {
  {"get_sample_size", lua_get_sample_size},
  {"get_sampling_rate", lua_get_sampling_rate},
  {"get_detection", lua_get_detection},
  {"pause_receiver", lua_pause_receiver},
  {"enable_receiver", lua_enable_receiver},
  {"queue_signal", lua_queue_signal},
  {"is_receiver_enabled", lua_is_receiver_enabled},
  {"is_transmitter_enabled", lua_is_transmitter_enabled},

  {NULL, NULL}
};

extern "C"
int luaopen_SoundComm (lua_State *L) {
  luaL_register(L, "SoundComm", sound_comm_lib);
  if (!init) {
    if (sound_comm_thread_init() != 0) {
      printf("SoundComm: error initializing.\n");
      exit(1);
    } 
    init = 1;
  }
  
  return 1;
}

