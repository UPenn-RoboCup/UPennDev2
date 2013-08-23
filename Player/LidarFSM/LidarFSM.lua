-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local lidarIdle = require'lidarIdle'
local lidarPan  = require'lidarPan'

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( lidarPan,lidarIdle )
--sm:add_state(lidarIdle)

-- Setup the transistions for this FSM
sm:set_transition( lidarPan,  'stop',  lidarIdle )
sm:set_transition( lidarIdle, 'start', lidarPan  )

local obj = {}
obj.entry = function()
  sm:entry()
end
obj.update = function()
  sm:update()
end
obj.update = function()
  sm:exit()
end

return obj