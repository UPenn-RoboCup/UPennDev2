--------------------------------
-- Approach Wire
-- (c) 2014 Stephen McGill
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local t_entry, t_update, t_finish
local timeout = 10.0
local get_time = Body.get_time

local pathIter
local qLArm_goal = Config.fsm.armInit.qLArm
local planner = require'libPlan'.new_planner(
  Body.Kinematics, Config.servo.min_rad, Config.servo.max_rad
)

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = get_time()
  t_update = t_entry
  t_finish = t
  -- Form the planner
  local qLArm = Body.get_larm_position()
  pathIter = planner:joint_iter(qLArm, qLArm_goal)
end

function state.update()
  --print(state._NAME..' Update' )
  local t  = get_time()
  local dt = t - t_update
  local dt_entry = t - t_entry
  if dt_entry>timeout then return'timeout' end
  -- Save this at the last update time
  t_update = t
  -- Find where we should go now
  local qLArm = Body.get_larm_position()
  local qLArm_wp = pathIter(qLArm)
  if not qLArm_wp then
    Body.set_larm_command_position(qLArm_goal)
    return'done'
  end
  -- Go to the waypoint
  Body.set_larm_command_position(qLArm_wp)
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
