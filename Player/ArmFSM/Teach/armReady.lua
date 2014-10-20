local state = {}
state._NAME = ...
require'hcm'
local Body = require'Body'
local K = Body.Kinematics
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local P = require'libPlan'
local t_entry, t_update, t_finish
local timeout = 10.0

local q_rGoal = Kinematics.inverse_r_arm_7(
	{0.46,-0.25, 0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
	Body.get_rarm_command_position(),
	0,
	0,
	Body.get_waist_command_position()
)

local q_lGoal = vector.zeros(#Body.get_larm_position())

local lPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]),
	vector.slice(Config.servo.max_rad, Config.parts.LArm[1], Config.parts.LArm[#Config.parts.LArm]))
local rPlanner = P.new_planner(K,
	vector.slice(Config.servo.min_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]), 
	vector.slice(Config.servo.max_rad, Config.parts.RArm[1], Config.parts.RArm[#Config.parts.RArm]))


local lPathIter, rPathIter

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
	--
	lPathIter = lPlanner:joint_iter(Body.get_larm_position(), q_lGoal, DEG_TO_RAD / 2)
	rPathIter = rPlanner:joint_iter(Body.get_rarm_position(), q_rGoal, DEG_TO_RAD / 2)
end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end
	-- Check the current for collisions
	--print('L Current', Body.get_larm_current()*1)
	--print('R Current', Body.get_rarm_current()*1)
	-- Plan the next joint position
  local q_lWaypoint = lPathIter(Body.get_larm_command_position())
	if type(q_lWaypoint)=='table' then
		Body.set_larm_command_position(q_lWaypoint)
	end
	local q_rWaypoint = rPathIter(Body.get_rarm_command_position())
	if type(q_rWaypoint)=='table' then
		Body.set_rarm_command_position(q_rWaypoint)
	end
	-- Check if done
	if q_lWaypoint==true and q_rWaypoint==true then
		return 'done'
	end
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
