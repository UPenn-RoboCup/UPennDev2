local state = {}
state._NAME = ...
local vector = require'vector'

local Body = require'Body'
local t_entry, t_update, t_finish
local timeout = 10.0
require'mcm'

local qLArm, qRArm

function state.entry()
  --print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  -- Torque OFF the motors
--  Body.set_larm_torque_enable(0)
--  Body.set_rarm_torque_enable(0)

  qLArm = Body.get_larm_position()
  qRArm = Body.get_rarm_position()

	--[[
		-- Set Hardware limits in case
  for i=1,3 do
    Body.set_larm_torque_enable(1)
    Body.set_rarm_torque_enable(1)
    Body.set_larm_command_velocity(500)
    Body.set_rarm_command_velocity(500)
    Body.set_larm_command_acceleration(50)
    Body.set_rarm_command_acceleration(50)
    Body.set_larm_position_p(8)
    Body.set_rarm_position_p(8)
    if not IS_WEBOTS then unix.usleep(1e5) end
  end
	--]]

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

  -- TODO: What if exit before first
  -- read request arrives?
  qLArm = Body.get_larm_position()
  qRArm = Body.get_rarm_position()

  Body.set_larm_command_position(qLArm)
  Body.set_rarm_command_position(qRArm)
  --print("LArm jangle:", qLArm * RAD_TO_DEG)

end

function state.exit()
  --print(state._NAME..' Exit' )
	-- Undo the hardware limits
  for i=1,3 do
		Body.set_larm_torque_enable(1)
		Body.set_rarm_torque_enable(1)
    Body.set_larm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_rarm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_larm_command_acceleration({200,200,200,200,200,200,200})
    Body.set_rarm_command_acceleration({200,200,200,200,200,200,200})
    Body.set_larm_position_p(32)
    Body.set_rarm_position_p(32)
    if not IS_WEBOTS then unix.usleep(1e5) end
  end

end

return state
