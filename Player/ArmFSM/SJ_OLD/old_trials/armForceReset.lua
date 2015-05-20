--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 15.0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  stage = 1
  Body.set_lgrip_percent(.7)
  Body.set_rgrip_percent(.7)
end

function state.update()
  Body.set_larm_command_position(Config.arm.qLArmPose1)
  Body.set_rarm_command_position(Config.arm.qRArmPose1)
  return"done";
 
end

function state.exit()
  Body.set_lgrip_percent(0.9)
  Body.set_rgrip_percent(0.9)
  print(state._NAME..' Exit' )
end


return state
