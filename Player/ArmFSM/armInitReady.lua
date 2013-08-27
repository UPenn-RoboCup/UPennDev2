--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = 'armInitReady'
local Config  = require'Config'
local Body    = require'Body'
local util    = require'util'
local t_entry, t_update, t_finish
local timeout = 15.0

-- Goal position is arm Init, with hands in front, ready to manipulate
local qLArmInit = Config.arm.qLArmInit[2]
local qRArmInit = Config.arm.qRArmInit[2]

-- Angular velocity limit
local dqArmMax = vector.new({10,10,10,15,45,45})*Body.DEG_TO_RAD

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  -- Where are the arms right now?
  local qLArm = Body.get_larm_position()
  local qRArm = Body.get_rarm_position()
  Body.set_larm_command_position(qLArm)
  Body.set_rarm_command_position(qRArm)

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  -- Left
  local qLArm = Body.get_larm_command_position()
  qLArm = util.approachTol( qLArm, qLArmInit, dqArmMax, dt )
  if qLArm~=true then
    Body.set_larm_command_position( qLArm )
  end
  -- Right
  local qRArm = Body.get_rarm_command_position()
  qRArm = util.approachTol( qRArm, qRArmInit, dqArmMax, dt )
  if qRArm~=true then
    Body.set_rarm_command_position( qRArm )
  end

  -- We are done when we are within tolerance
  if qLArm==true and qRArm==true then return 'done' end

end

function state.exit()
  print(state._NAME..' Exit' )
end

-- Add Epi-sate
state.epi = {}
-- Is this going going in forward to Ready, or reverse to Init?
state.epi.reverse = false

return state
