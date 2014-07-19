local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {};
local shsize = {};

--SJ: Let's use this SHM for robocup behavior testing

-- 0 for initial / 1 for ready 
-- 2 for set / 3 for play / 4 fin
-- 5: Pre-initialized (idle)
-- 6 for testing

shared.game = {};
shared.game.state = vector.zeros(1);

-- 0: goalie   1: attacker   2:Force stop and go to Testing
shared.game.role = vector.ones(1)

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
