local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {};
local shsize = {};

shared.game = {};
shared.game.state = vector.zeros(1);
-- 0: goalie   1: attacker
shared.game.role = vector.ones(1)

--[[
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
--]]

-- Keep track of every state machine
-- Use the Config'd FSMs
shared.fsm = {}
if Config and Config.fsm then
  for _,sm in ipairs(Config.fsm.enabled) do
    shared.fsm[sm] = ''
  end
end

-- Call the initializer
memory.init_shm_segment(..., shared, shsize)