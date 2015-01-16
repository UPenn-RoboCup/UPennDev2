--Stationary kick state

local state = {}
state._NAME = ...

local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local moveleg = require'moveleg'
require'mcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

---------------------------
-- State machine methods --
---------------------------
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  mcm.set_walk_bipedal(1)
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

  local qWaist = Body.get_waist_command_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()    
  local uTorso = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  mcm.set_walk_ismoving(0) --We stopped moving

  --Adjust body height
  local bodyHeight_now = mcm.get_stance_bodyHeight()  
  local bodyHeightTarget = hcm.get_motion_bodyHeightTarget()
  bodyHeightTarget = math.max(0.75,math.min(Config.walk.bodyHeight,bodyHeightTarget))

  local bodyHeight = util.approachTol( bodyHeight_now, 
    bodyHeightTarget, Config.stance.dHeight, t_diff )
  
  local zLeft,zRight = 0,0
  supportLeg = 2; --Double support


  local gyro_rpy = Body.get_gyro()

  local delta_legs
  delta_legs, angleShift = moveleg.get_leg_compensation_simple(supportLeg,0,gyro_rpy, angleShift)

--Compensation for arm / objects
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local uTorsoCompensated = util.pose_global(
     {uTorsoComp[1],uTorsoComp[2],0},uTorso)
  mcm.set_stance_bodyHeight(bodyHeight)  
  moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight,  
    0,0,delta_legs)
end 

function state.exit()
  print(state._NAME..' Exit')
end

return state