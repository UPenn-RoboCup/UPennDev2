local state = {}
state._NAME = ...

--motionInit: initialize legs to correct position


require'mcm'
require'hcm'
require'wcm'
local Body       = require'Body'
local K          = Body.Kinematics
local util       = require'util'
local vector     = require'vector'
local timeout    = 20.0
local t_readings = 0.20
local t_settle   = 0.10
local t_entry, t_update, t_finish
local moveleg = require'moveleg'


local err_th = 1*DEG_TO_RAD
if IS_WEBOTS then err_th = 5*DEG_TO_RAD end
-- Set the desired waist
local qWaist_desired = Config.stance.qWaist

local pTorso, qLLeg, qRLeg

local stage = 1
local t_last_debug
local last_error = 999

local bodyHeight = 1.0


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t_entry
  t_last_debug=t_entry
end

--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
  -- Get the time of update
  local t  = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  
  uLeft = mcm.get_status_uLeft()
  uRight = mcm.get_status_uRight()
  uTorso = mcm.get_status_uTorso()  
  local zLeg = mcm.get_status_zLeg()
  
  local bodyHeight_now = mcm.get_stance_bodyHeight()
  local bodyHeightTarget = 0.80 --lower torso

  bodyHeightTarget = math.max(0.75,math.min(Config.walk.bodyHeight,bodyHeightTarget))
  local bodyHeight,done = util.approachTol( bodyHeight_now, bodyHeightTarget, Config.stance.dHeight, t_diff )

  local gyro_rpy = Body.get_gyro()
  mcm.set_stance_bodyHeight(bodyHeight)  
  moveleg.ft_compensate(t_diff)
  local delta_legs, angleShift = moveleg.get_leg_compensation_new(supportLeg,0,gyro_rpy, angleShift,t_diff)
  moveleg.set_leg_positions(delta_legs)
 
  if done then 
    print("motion uninit completed")
    return "done"
  end
end

function state.exit()
  mcm.set_walk_ismoving(0) --We are stopped
  wcm.set_robot_initdone(0)
end

return state
