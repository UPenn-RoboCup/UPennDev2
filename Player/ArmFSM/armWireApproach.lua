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

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = get_time()
  t_update = t_entry
  t_finish = t
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
  -- Find the kinematics
  local fkLArm = K.forward_arm(qLArm)
  -- Step in the direction of the gripper (z for youbot...)
  local fkLArm_next = fkLArm * T.trans(0, 0, ds)
  local iqArm_next = vector.new(K.inverse_arm(desired_tr, qLArm))
  -- TODO: Add small changes on the local camera roll and camera yaw
  -- These are independent of the IK in the local z direction
  iqArm_next[1] = qLArm[1]
  iqArm_next[5] = qLArm[5]
  -- Go to the waypoint
  Body.set_larm_command_position(iqArm_next)
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
