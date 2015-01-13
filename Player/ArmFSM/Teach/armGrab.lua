local state = {}
state._NAME = ...
local Body = require'Body'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local T = require'Transform'
local K = require'K_ffi'
local t_entry, t_update, t_finish
local timeout = 10.0

local lPathIter, use_feedback, stage

local function get_local_cylinder()
  local cyl = hcm.get_assist_cylinder()
  local xc, yc, zc, r, h = unpack(cyl)
  local globalCyl = T.trans(xc, yc, zc)
  local globalTorso = T.from_rpy_trans(Body.get_rpy(), {0,0,mcm.get_walk_bodyHeight()})
  return T.inv(globalTorso) * globalCyl
end

local function alignTable()
  local localCyl = get_local_cylinder()
  local fkLArm = K.forward_larm(Body.get_larm_position())
  local xc,yc,zc = unpack(T.position6D(localCyl))
  local x,y,z = unpack(T.position6D(fkLArm))
  local trLGoal = T.transform6D{x, y, zc, 0, 0, -90*DEG_TO_RAD}
  return movearm.goto_tr_via_q(trLGoal, nil, {trLGoal[2][4] > 0.2 and 20*DEG_TO_RAD or 70*DEG_TO_RAD}), true
end

local function faceDrill()
  local localCyl = get_local_cylinder()
  local fkLArm = K.forward_larm(Body.get_larm_position())
  local xc,yc,zc = unpack(T.position6D(localCyl))
  local x,y,z = unpack(T.position6D(fkLArm))
  local trLGoal = T.transform6D{xc, yc + 0.10, zc, 0, 0, -90*DEG_TO_RAD}
  return movearm.goto_tr(trLGoal, nil, {yc > 0.2 and -10*DEG_TO_RAD or 30*DEG_TO_RAD}), true
end

local function go2drill()
  local localCyl = get_local_cylinder()
  local fkLArm = K.forward_larm(Body.get_larm_position())
  local xc,yc,zc = unpack(T.position6D(localCyl))
  local x,y,z = unpack(T.position6D(fkLArm))
  local trLGoal = T.transform6D{xc, yc, zc, 0, 0, -90*DEG_TO_RAD}
  return movearm.goto_tr(trLGoal, nil, {yc > 0.2 and -10*DEG_TO_RAD or 30*DEG_TO_RAD}), true
end

local stage2iter = {
  alignTable,
  faceDrill,
  go2drill
}

function state.entry()
  print(state._NAME..' Entry')
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  --
  stage = 1
  lPathIter = stage2iter[stage]()
end

function state.update()
--  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  if t-t_entry > timeout then return'timeout' end
  local qLArm, moreL, q_lWaypoint
  if use_feedback then
    qLArm = Body.get_larm_position()
    moreL, q_lWaypoint = lPathIter(qLArm)
  else
  	qLArm = Body.get_larm_command_position()
  	moreL, q_lWaypoint = lPathIter(qLArm, dt)
  end
	Body.set_larm_command_position(q_lWaypoint)
  -- Check if done
  if not moreL then
    stage = stage + 1
    if stage>#stage2iter then return'done' end
    lPathIter, use_feedback = stage2iter[stage]()
  end
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
