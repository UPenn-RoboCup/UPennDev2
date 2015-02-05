--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local T = require'Transform'
local movearm = require'movearm'

local t_entry, t_update, t_finish
local timeout = 30.0

--[[
local trLGoal = T.transform6D{0.1, 0.3, -0.3, 0, 30*DEG_TO_RAD, -45*DEG_TO_RAD}
local trRGoal = T.transform6D{0.1, -0.32, -0.28, 15*DEG_TO_RAD, 35*DEG_TO_RAD, 70*DEG_TO_RAD}
--]]
-- From the IK solution above, in webots
local qLGoal = vector.new{140.055, 14.5738, 5, -82.3928, 65.0266, 38.4997, -59.7267} * DEG_TO_RAD
local qRGoal = vector.new{157.166, -40.6751, -5, -101.136, -55.7423, -26.6821, 63.5934} * DEG_TO_RAD

local shoulderLGoal, shoulderRGoal = 5*DEG_TO_RAD, -5*DEG_TO_RAD

local lPathIter, rPathIter
local setShoulderYaw

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  -- To get to the IK solution
  --lPathIter, rPathIter, qLGoal, qRGoal = movearm.goto_tr_via_q(trLGoal, trRGoal, {shoulderLGoal}, {shoulderRGoal})
  -- Given the IK solution
  lPathIter, rPathIter = movearm.goto_q(qLGoal, qRGoal)
  if NO_YAW_FIRST then
    setShoulderYaw = true
  else
    -- First, ignore the shoulderYaw, since it can cause issues of self collision
    local qL = Body.get_larm_position()
    local qR = Body.get_rarm_position()
    qL[3] = -20*DEG_TO_RAD
    qR[3] = 20*DEG_TO_RAD
    lPathIter, rPathIter = movearm.goto_q(qL, qR)
    setShoulderYaw = false
  end

	-- Set Hardware limits in case
  for i=1,5 do
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
  if t-t_entry > timeout then return'timeout' end

	-- Timing necessary
	----[[
	local qLArm = Body.get_larm_command_position()
	local moreL, q_lWaypoint = lPathIter(qLArm, dt)
	--]]
	-- No time needed
	--[[
	local qLArm = Body.get_larm_position()
	local moreL, q_lWaypoint = lPathIter(qLArm)
	--]]
	Body.set_larm_command_position(q_lWaypoint)

	----[[
	local qRArm = Body.get_rarm_command_position()
	local moreR, q_rWaypoint = rPathIter(qRArm, dt)
	--]]
	--[[
	local qRArm = Body.get_rarm_position()
	local moreR, q_rWaypoint = rPathIter(qRArm)
	--]]
	Body.set_rarm_command_position(q_rWaypoint)
	-- Check if done
	if not moreL and not moreR then
    print('setShoulderYaw', setShoulderYaw)
    if setShoulderYaw then
  		return 'done'
    else
			-- ignore sanitization for the init position, which is absolutely known
      lPathIter, rPathIter = movearm.goto_q(qLGoal, qRGoal, 3*DEG_TO_RAD, true)
      setShoulderYaw = true
    end
	end

end

function state.exit()

	-- Undo the hardware limits
  for i=1,5 do
    Body.set_larm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_rarm_command_velocity({17000,17000,17000,17000,17000,17000,17000})
    Body.set_larm_command_acceleration({200,200,200,200,200,200,200})
    Body.set_rarm_command_acceleration({200,200,200,200,200,200,200})
    Body.set_larm_position_p(32)
    Body.set_rarm_position_p(32)
    if not IS_WEBOTS then unix.usleep(1e5) end
  end

  print(state._NAME..' Exit' )
end

return state
