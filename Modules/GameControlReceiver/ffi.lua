local libGC = {}
local ffi = require'ffi'
local udp = require'udp'
local GC_TIMEOUT = 1 / 10

local gc_constants = [[

#define GAMECONTROLLER_PORT            3838

#define GAMECONTROLLER_STRUCT_HEADER   "RGme"
#define GAMECONTROLLER_STRUCT_VERSION  8

#define MAX_NUM_PLAYERS             11

#define TEAM_BLUE                   0
#define TEAM_CYAN                   0
#define TEAM_RED                    1
#define TEAM_MAGENTA                1
#define DROPBALL                    2

#define STATE_INITIAL               0
#define STATE_READY                 1
#define STATE_SET                   2
#define STATE_PLAYING               3
#define STATE_FINISHED              4

#define STATE2_NORMAL               0
#define STATE2_PENALTYSHOOT         1
#define STATE2_OVERTIME             2
#define STATE2_TIMEOUT              3

]]

libGC.state_to_name = {
  [0] = 'init',
  [1] = 'ready',
  [2] = 'set',
  [3] = 'play',
  [4] = 'finish',
}

gc_constants = gc_constants..[[

#define PENALTY_NONE                        0
// SPL
#define PENALTY_SPL_BALL_HOLDING            1
#define PENALTY_SPL_PLAYER_PUSHING          2
#define PENALTY_SPL_OBSTRUCTION             3
#define PENALTY_SPL_INACTIVE_PLAYER         4
#define PENALTY_SPL_ILLEGAL_DEFENDER        5
#define PENALTY_SPL_LEAVING_THE_FIELD       6
#define PENALTY_SPL_PLAYING_WITH_HANDS      7
#define PENALTY_SPL_REQUEST_FOR_PICKUP      8
#define PENALTY_SPL_COACH_MOTION            9
// HL Kid Size
#define PENALTY_HL_KID_BALL_MANIPULATION    1
#define PENALTY_HL_KID_PHYSICAL_CONTACT     2
#define PENALTY_HL_KID_ILLEGAL_ATTACK       3
#define PENALTY_HL_KID_ILLEGAL_DEFENSE      4
#define PENALTY_HL_KID_REQUEST_FOR_PICKUP   5
#define PENALTY_HL_KID_REQUEST_FOR_SERVICE  6
#define PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE 7
// HL Teen Size
#define PENALTY_HL_TEEN_BALL_MANIPULATION   1
#define PENALTY_HL_TEEN_PHYSICAL_CONTACT    2
#define PENALTY_HL_TEEN_ILLEGAL_ATTACK      3
#define PENALTY_HL_TEEN_ILLEGAL_DEFENSE     4
#define PENALTY_HL_TEEN_REQUEST_FOR_PICKUP  5
#define PENALTY_HL_TEEN_REQUEST_FOR_SERVICE 6
#define PENALTY_HL_TEEN_REQUEST_FOR_PICKUP_2_SERVICE 7

#define PENALTY_SUBSTITUTE                  14
#define PENALTY_MANUAL                      15

// data structure header
#define GAMECONTROLLER_RETURN_STRUCT_HEADER      "RGrt"
#define GAMECONTROLLER_RETURN_STRUCT_VERSION     2

#define GAMECONTROLLER_RETURN_MSG_MAN_PENALISE   0
#define GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE 1
#define GAMECONTROLLER_RETURN_MSG_ALIVE          2

#define SPL_COACH_MESSAGE_PORT           3839

#define SPL_COACH_MESSAGE_STRUCT_HEADER  "SPLC"
#define SPL_COACH_MESSAGE_STRUCT_VERSION 2
#define SPL_COACH_MESSAGE_SIZE           40

]]

local gc_structs = [[
struct RobotInfo
{
  uint8_t penalty;              // penalty state of the player
  uint8_t secsTillUnpenalised;  // estimate of time till unpenalised
};

struct TeamInfo
{
  uint8_t teamNumber;           // unique team number
  uint8_t teamColour;           // colour of the team
  uint8_t score;                // team score
  uint8_t penaltyShot;          // penalty shot counter
  uint16_t singleShots;         // bits represent penalty shot success
  uint8_t message[SPL_COACH_MESSAGE_SIZE]; // the coach message to the team
  struct RobotInfo coach;
  struct RobotInfo players[MAX_NUM_PLAYERS]; // the players of the team
};

struct RoboCupGameControlData
{
  char header[4];               // header to identify the structure
  uint8_t version;              // version of the data structure
  uint8_t packetNumber;         // number incremented with each packet sent (with wraparound)
  uint8_t playersPerTeam;       // The number of players on a team
  uint8_t state;                // state of the game (STATE_READY, STATE_PLAYING, etc)
  uint8_t firstHalf;            // 1 = game in first half, 0 otherwise
  uint8_t kickOffTeam;          // the next team to kick off (TEAM_BLUE, TEAM_RED)
  uint8_t secondaryState;       // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
  uint8_t dropInTeam;           // team that caused last drop in
  uint16_t dropInTime;          // number of seconds passed since the last drop in.  -1 before first dropin
  int16_t secsRemaining;       // estimate of number of seconds remaining in the half
  uint16_t secondaryTime;       // number of seconds shown as secondary time (remaining ready, until free ball, etc)
  struct TeamInfo teams[2];
};

struct RoboCupGameControlReturnData
{
  char header[4];
  uint8_t version;
  uint8_t team;    // team number
  uint8_t player;  // player number starts with 1
  uint8_t message; // one of the three messages defined above
};

struct SPLCoachMessage
{
  char header[4];        // "SPLC"
  uint8_t version;       // SPL_COACH_MESSAGE_STRUCT_VERSION
  uint8_t team;          // team number

  // buffer for message
  uint8_t message[SPL_COACH_MESSAGE_SIZE];
};

]]

local function define_to_vars(define, tbl)
  local define_iterator = "#define (%S+)%s+(%S+)"
  local defines = tbl or {}
  for name, value in define:gmatch(define_iterator) do
    defines[name] = tonumber(value) or value:match('"(%S+)"')
  end
  return defines
end

local function def_structs(structs, defines, tbl)
  local constants
  if defines then
    constants = define_to_vars(defines, tbl)
    for k, v in pairs(constants) do
      structs = structs:gsub(k, v)
    end
  end
  --print(structs)
  ffi.cdef(structs)
  return constants
end

def_structs(gc_structs, gc_constants, libGC)

local function send_coach(self, msg)
  if #msg > libGC.SPL_COACH_MESSAGE_SIZE then
    msg = msg:sub(1, libGC.SPL_COACH_MESSAGE_SIZE)
  end
  local pkt = ffi.new('struct SPLCoachMessage', {
    header = libGC.SPL_COACH_MESSAGE_STRUCT_HEADER,
    version = libGC.SPL_COACH_MESSAGE_STRUCT_VERSION,
    team = self.team or 0,
    message = msg,
  })
  return self.sender:send(ffi.string(pkt, ffi.sizeof(pkt)))
end

local function send_return(self)
  local pkt = ffi.new('struct RoboCupGameControlReturnData', {
    header = libGC.GAMECONTROLLER_RETURN_STRUCT_HEADER,
    version = libGC.GAMECONTROLLER_RETURN_STRUCT_VERSION,
    team = self.team or 0,
    player = self.player or 0,
    message = libGC.GAMECONTROLLER_RETURN_MSG_ALIVE,
  })
  return self.sender:send(ffi.string(pkt, ffi.sizeof(pkt)))
end

local gc_data = ffi.new('struct RoboCupGameControlData')
local function receive(self, buf)
  buf = buf or self.receiver:receive()
  if not buf then return end
  local sz = #buf
  if sz==ffi.sizeof('struct RoboCupGameControlData') then
		ffi.copy(gc_data, buf, sz)
    return gc_data
  elseif sz==ffi.sizeof('struct RoboCupGameControlReturnData') then
    -- ignore
  elseif sz==ffi.sizeof('struct SPLCoachMessage') then
    -- ignore
  end
end

local function wait_for_gc(self)
  local fd = self.receiver:descriptor()
  local status, ready = unix.select({fd}, GC_TIMEOUT)
  return self:receive()
end

function libGC.init(team, player, gc_addr)
  local udp_sender = udp.new_sender(gc_addr or 'localhost', libGC.GAMECONTROLLER_PORT)
  local udp_receiver = udp.new_receiver(libGC.GAMECONTROLLER_PORT)
  local gc = {
    send_return = send_return,
    send_coach = send_coach,
    receive = receive,
    wait_for_gc = wait_for_gc,
    receiver = udp_receiver,
    sender = udp_sender,
    player = player,
    team = team,
  }
  return gc
end

return libGC
