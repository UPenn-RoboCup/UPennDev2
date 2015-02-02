local state = {}
state._NAME = ...
local Body = require'Body'
local movearm = require'movearm'

local t_entry, t_update, t_finish
local timeout = 10.0
local lPathIter, rPathIter
local qLGoal, qRGoal

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  -- Reset the human position
  hcm.set_teleop_larm(Body.get_larm_position())
  hcm.set_teleop_rarm(Body.get_rarm_position())
  qLGoal = hcm.get_teleop_larm()
  qRGoal = hcm.get_teleop_rarm()
  lPathIter, rPathIter = movearm.goto_q(qLGoal, qRGoal)
end

function state.update()
	--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end
  
  -- See if commanded a new position
  if qLGoal~=hcm.get_teleop_larm() or qRGoal~=hcm.get_teleop_rarm() then
    -- Get the goal from hcm
    qLGoal = hcm.get_teleop_larm()
    qRGoal = hcm.get_teleop_rarm()
    -- Make the iterators
    lPathIter, rPathIter = movearm.goto_q(qLGoal, qRGoal)
  end
  
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
	Body.set_larm_command_position(moreL and q_lWaypoint or qLGoal)
	
	----[[
	local qRArm = Body.get_rarm_command_position()
	local moreR, q_rWaypoint = rPathIter(qRArm, dt)
	--]]
	--[[
	local qRArm = Body.get_rarm_position()
	local moreR, q_rWaypoint = rPathIter(qRArm)
	--]]
	Body.set_rarm_command_position(moreR and q_rWaypoint or qRGoal)
  
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
