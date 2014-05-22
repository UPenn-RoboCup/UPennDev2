--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local t_entry, t_update, t_finish
local timeout = 10.0
local get_time = Body.get_time

local qLArm_goal = Config.fsm.armStance.qLArm

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = get_time()
  t_update = t_entry
  t_finish = t
  -- Just set our position
  Body.set_larm_command_position(qLArm_goal)
end

function state.update()
  local t  = get_time()
  local dt = t - t_update
  local dt_entry = t - t_entry
  if dt_entry>timeout then return'timeout' end
  -- Save this at the last update time
  t_update = t
  -- Find where we should go now
  local qLArm = Body.get_larm_position()
  -- Check if we are too far from where we wish to be
  local sum = 0
  for _,v in ipairs(qLArm - qLArm_goal) do sum = sum + v end
  if sum>5*DEG_TO_RAD then return'far' end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
