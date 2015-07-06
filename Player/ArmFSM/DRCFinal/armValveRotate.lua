local state = {}
state._NAME = ...
local vector = require'vector'

local Body = require'Body'
local t_entry, t_update, t_finish
local timeout = 10.0
require'mcm'

local qYaw0
local qLArm0
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

   qLArm0 = Body.get_larm_command_position()
   hcm.set_teleop_steering(0)
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  local steering = math.min(math.pi*2.5, math.max(-2.5*math.pi, hcm.get_teleop_steering() ))

  local qLArm = util.shallow_copy(qLArm0)
  qLArmT = qLArm0[7]+steering
 
  local qLArmC = Body.get_larm_command_position()
  qLArm[7] = util.approachTol(qLArmC[7], qLArmT, math.pi/2, dt)

  Body.set_larm_command_position(qLArm)
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
