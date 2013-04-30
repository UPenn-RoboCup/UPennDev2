local shm = require('shm');
local util = require('util');
local vector = require('vector');
local Config = require('Config');
local Speak = require('Speak');

-- shared properties
local shared = {};
local shsize = {};

shared.game = {};
shared.game.state = vector.zeros(1);
shared.game.nplayers = vector.zeros(1);
shared.game.kickoff = vector.zeros(1);
shared.game.half = vector.zeros(1);
shared.game.penalty = vector.zeros(Config.game.nPlayers);
shared.game.opponent_penalty = vector.zeros(Config.game.nPlayers);
shared.game.time_remaining = vector.zeros(1);
shared.game.last_update = vector.zeros(1);

shared.game.paused = vector.zeros(1);
shared.game.gc_latency = vector.zeros(1);--GC message latency
shared.game.tm_latency = vector.zeros(1);--Team message latency

shared.game.our_score = vector.zeros(1);
shared.game.opponent_score = vector.zeros(1);

shared.team = {};
shared.team.number = vector.zeros(1);
shared.team.player_id = vector.zeros(1);
shared.team.color = vector.zeros(1);
shared.team.role = vector.zeros(1);

shared.team.forced_role = vector.zeros(1); --for role testing

--for double pass
shared.team.task_state = vector.zeros(2); 
shared.team.target = vector.zeros(3);
shared.team.balltarget = vector.zeros(3);

shared.fsm = {};
shared.fsm.body_state = '';
shared.fsm.head_state = '';
shared.fsm.motion_state = '';
shared.fsm.game_state = '';

util.init_shm_segment(..., shared, shsize);

-- initialize player id
gcm.set_team_player_id(Config.game.playerID);

-- initialize team id
gcm.set_team_number(Config.game.teamNumber);

-- Initialize nPlayers
gcm.set_game_nplayers(Config.game.nPlayers);

-- initialize state to 'initial'
gcm.set_game_state(0);
gcm.set_team_role(Config.game.role);

-- helper functions
function gcm.in_penalty()
  return gcm.get_game_penalty()[gcm.get_team_player_id()] > 0;
end

function gcm.say_id()
  Speak.talk('Player ID '..Config.game.playerID);
  Speak.talk('Team Number '..Config.game.teamNumber);
end



