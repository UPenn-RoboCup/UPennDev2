--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 10.0

local lPathIter, rPathIter

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

	lPathIter, rPathIter = movearm.goto_wrists(
		{Config.arm.pLWristTarget0, Config.arm.lShoulderYaw0, Config.arm.lrpy0},
		{Config.arm.pRWristTarget0, Config.arm.rShoulderYaw0, Config.arm.rrpy0}
	)
	
	-- Set Hardware limits in case
  for i=1,10 do
    Body.set_larm_command_velocity(500)
    Body.set_rarm_command_velocity(500)
    Body.set_larm_command_acceleration(50)
    Body.set_rarm_command_acceleration(50)
    Body.set_larm_position_p(8)
    Body.set_rarm_position_p(8)
    if not IS_WEBOTS then unix.usleep(1e5) end
  end
end

function state.update()
	--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
	--[[
	local moreL, q_lWaypoint = lPathIter(Body.get_larm_command_position(), dt)
	Body.set_larm_command_position(q_lWaypoint)
	local moreR, q_rWaypoint = rPathIter(Body.get_rarm_command_position(), dt)
	Body.set_rarm_command_position(q_rWaypoint)
	--]]
	local qL = Body.get_larm_position()
	local moreL, q_lWaypoint = lPathIter(qL)
	Body.set_larm_command_position(q_lWaypoint)
	local qR = Body.get_rarm_position()
	local moreR, q_rWaypoint = rPathIter(qR)
	Body.set_rarm_command_position(q_rWaypoint)
	-- Check if done
	if not moreL and not moreR then return 'done' end

end

function state.exit()

	-- Undo the hardware limits
  for i=1,10 do
    Body.set_larm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_rarm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_larm_command_acceleration({200,200,200,200,200,200,200})
    Body.set_rarm_command_acceleration({200,200,200,200,200,200,200})
    --Body.set_larm_position_p(32)
    --Body.set_rarm_position_p(32)
    if not IS_WEBOTS then unix.usleep(1e5) end
  end

  print(state._NAME..' Exit' )
end

return state
