-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states
local motionRelax  = require'motionRelax'
local motionStance = require'motionStance'
local motionWalk   = require(Config.dev.walk)
--[[
local motionSit    = require'motionSit'
--]]

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( motionWalk, motionRelax, motionStance  )

-- Setup the transistions for this FSM
sm:set_transition(motionRelax, 'standup', motionStance)
sm:set_transition(motionRelax, 'timeout', motionRelax)
--
sm:set_transition(motionStance, 'done',  motionRelax)
sm:set_transition(motionStance, 'timeout',  motionStance)
sm:set_transition(motionStance, 'walk', motionWalk)
--sm:set_transition(motionStance, 'sit',  motionSit)
--[[
sm:set_transition(motionWalk, 'sit',    motionSit)
sm:set_transition(motionWalk, 'stance', motionStance)
--
sm:set_transition(motionSit, 'done',    motionRelax)
sm:set_transition(motionSit, 'standup', motionStance)
--]]

return sm