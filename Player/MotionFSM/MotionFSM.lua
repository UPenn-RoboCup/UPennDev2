-- Config guides special situations
local Config = require'Config'
-- Use the fsm module
local fsm = require'fsm'

-- Require the needed states

local motionIdle   = require'motionIdle' --Initial state, all legs torqued off
local motionInit   = require'motionInit' --Torque on legs and go to initial leg position
local motionBiasInit   = require'motionBiasInit' --Torque on legs and go to initial leg position
local motionStance = require'motionStance' --Robots stands still, balancing itself, ready for walk again
local motionHeightReturn = require'motionHeightReturn' --Robots stands still, balancing itself, ready for walk again
local motionSit    = require'motionSit' --Robots changes the body height for manipulation
local motionUnSit  = require'motionUnSit' --Robots changes the body height for manipulation
local motionWalk   = require(Config.dev.walk) --Reactive walking
local motionStep   = require'motionStep'   --Stationary stepping
local motionStepPreview   = require'motionStepPreview' --ZMP preview stepping
local motionStepNonstop = require'motionStepNonstop'


--Our robot never fall!
--local motionFall   = require'motionFall' 

-- Instantiate a new state machine with an initial state
-- This will be returned to the user
local sm = fsm.new( motionIdle, motionInit, motionStance)
--, motionFall )
sm:add_state(motionWalk)
sm:add_state(motionBiasInit)
sm:add_state(motionStep)
sm:add_state(motionStepNonstop)
sm:add_state(motionStepPreview)
sm:add_state(motionSit)
sm:add_state(motionUnSit)
sm:add_state(motionHeightReturn)


sm:set_transition(motionIdle, 'stand', motionInit)
sm:set_transition(motionInit, 'done', motionStance)

sm:set_transition(motionIdle, 'bias', motionBiasInit)
sm:set_transition(motionBiasInit, 'bias', motionInit)
sm:set_transition(motionStance, 'bias', motionBiasInit)

--motionstance change bodyheight to target height
--And it does balancing and torso compensation as well
sm:set_transition(motionStance, 'sit', motionSit)

--for step, we change back to initial bodyheight and start stepping
sm:set_transition(motionStance, 'preview', motionHeightReturn)
sm:set_transition(motionHeightReturn, 'done', motionStepPreview)


--We don't use regular walk any more
--sm:set_transition(motionStance, 'step', motionStep)

--We keep this for webots only
if IS_WEBOTS then
  sm:set_transition(motionStance, 'walk', motionWalk)
end

sm:set_transition(motionSit, 'stand', motionUnSit)
sm:set_transition(motionUnSit, 'done', motionStance)

--Walk stop are handled by HCM variable
--As it can stop walking mid-step
sm:set_transition(motionWalk, 'done', motionStance)
sm:set_transition(motionStep, 'done', motionStance)
sm:set_transition(motionStepPreview, 'done', motionStance)


--We don't use non-stopping transition here
--sm:set_transition(motionWalk, 'done_step', motionStepNonstop)
sm:set_transition(motionStepNonstop, 'done', motionStance)
sm:set_transition(motionStepNonstop, 'towalk', motionWalk)


-- Add "special events" like 'footstep' that set
-- a variable number of footsteps (not suitable for shm)
-- Msgpacked special events
local mp = require'msgpack'
local special_evts = {
--[[
  preview = function(extra)
    local feet = mp.unpack(extra)
    --print('feet',unpack(feet))
    --motionStepPreview.make_step_queue(feet)
  end,
--]]  
  stand = function()
    mcm.set_walk_stoprequest(1)
  end,
  stepnonstop = function()
    mcm.set_walk_steprequest(1)
  end
}
--------------------------
-- Setup the FSM object --
--------------------------
local obj = {}
obj._NAME = ...
local util = require'util'
-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber(...,true)
local debug_str = util.color(obj._NAME..' Event:','green')
local special_evts = special_evts or {}
obj.entry = function()
  sm:entry()
end
obj.update = function()
  -- Check for out of process events in non-blocking fashion
  local event, has_more = evts:receive(true)
  if event then
    local debug_tbl = {debug_str,event}
    if has_more then
      extra = evts:receive(true)
      table.insert(debug_tbl,'Extra')
    end
    local special = special_evts[event]
    if special then
      special(extra)
      table.insert(debug_tbl,'Special!')
    end
    print(table.concat(debug_tbl,' '))
    sm:add_event(event)
  end
  sm:update()
end
obj.exit = function()
  sm:exit()
end

obj.sm = sm

return obj