local state = {}
state._NAME = ...
local vector = require'vector'

local Body = require'Body'
local t_entry, t_update, t_finish
local timeout = 10.0
require'mcm'

local qLArm, qRArm

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  -- Torque OFF the motors
--  Body.set_larm_torque_enable(0)
--  Body.set_rarm_torque_enable(0)

Body.set_rgrip_mode('torque')
Body.set_rgrip_torque_enable(1)
Body.set_rgrip_mode('torque')
Body.set_rgrip_torque_enable(1)
Body.set_rgrip_mode('torque')

  qLArm = Body.get_larm_position()
  qRArm = Body.get_rarm_position()

  mcm.set_arm_handoffset(Config.arm.handoffset.gripper)
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  -- TODO: What if exit before first
  -- read request arrives?
  qLArm = Body.get_larm_position()
  qRArm = Body.get_rarm_position()

  Body.set_larm_command_position(qLArm)
  Body.set_rarm_command_position(qRArm)
  --print("LArm jangle:",vector.new(qLArm)*RAD_TO_DEG)

end

function state.exit()
  print(state._NAME..' Exit' )
  Body.set_larm_torque_enable(1)
  Body.set_rarm_torque_enable(1)
end

return state
