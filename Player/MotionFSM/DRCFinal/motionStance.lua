--Stance state is basically a Walk controller
--Without any torso or feet update
--We share the leg joint generation / balancing code 
--with walk controllers

local state = {}
state._NAME = ...

local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local moveleg = require'moveleg'
local libStep = require'libStep'
require'mcm'
require'hcm'

-- Keep track of important times
local t_entry, t_update, t_last_step

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}
local uLeft,uRight,uTorso
local zLeft,zRight = 0,0

---------------------------
-- State machine methods --
---------------------------


local vel_movement = 0.02 --1cm per sec
local vel_lift = 0.03 --1cm per sec

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_walk_t_last(t_entry)

  mcm.set_walk_bipedal(1)
  mcm.set_walk_steprequest(0)
  mcm.set_motion_state(2)


  uLeft = mcm.get_status_uLeft()
  uRight = mcm.get_status_uRight()
  uTorso = mcm.get_status_uTorso()  
  local zLeg = mcm.get_status_zLeg()
  print("zLeg:",unpack(zLeg))

  hcm.set_legdebug_left({uLeft[1],uLeft[2],uLeft[3],zLeg[1]})
  hcm.set_legdebug_right({uRight[1],uRight[2],uRight[3],zLeg[2]})
  hcm.set_legdebug_torso({uTorso[1],uTorso[2]})
  hcm.set_legdebug_torso_angle({0,0})
  hcm.set_legdebug_enable_balance({0,0})

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
  local zLeg = mcm.get_status_zLeg()

  --Adjust body height
  local bodyHeight_now = mcm.get_stance_bodyHeight()
  local bodyHeightTarget = hcm.get_motion_bodyHeightTarget()
  bodyHeightTarget = math.max(0.75,math.min(Config.walk.bodyHeight,bodyHeightTarget))

  local bodyHeight = util.approachTol( bodyHeight_now, bodyHeightTarget, Config.stance.dHeight, t_diff )
  
  supportLeg = 2; --Double support


  local gyro_rpy = Body.get_gyro()
  local delta_legs

------------------------------------------------
--External control
  local uLeftTarget = hcm.get_legdebug_left() 
  local uRightTarget = hcm.get_legdebug_right()  
  local uTorsoTarget = hcm.get_legdebug_torso()  
  
  uLeft[1] = util.approachTol( uLeft[1],uLeftTarget[1],vel_movement , t_diff )
  uLeft[2] = util.approachTol( uLeft[2],uLeftTarget[2],vel_movement , t_diff )

  uRight[1] = util.approachTol( uRight[1],uRightTarget[1],vel_movement , t_diff )
  uRight[2] = util.approachTol( uRight[2],uRightTarget[2],vel_movement , t_diff )

  uTorso[1] = util.approachTol( uTorso[1],uTorsoTarget[1],vel_movement , t_diff )
  uTorso[2] = util.approachTol( uTorso[2],uTorsoTarget[2],vel_movement , t_diff )

  local enable_balance = hcm.get_legdebug_enable_balance()
  if enable_balance[1]+enable_balance[2]>0 then
    uLeftTarget[4] = zLeg[1]
    uRightTarget[4] = zLeg[2]
    hcm.set_legdebug_left(uLeftTarget) 
    hcm.set_legdebug_right(uRightTarget) 
  else
    zLeg[1] = util.approachTol( zLeg[1],uLeftTarget[4],vel_lift , t_diff )
    zLeg[2] = util.approachTol( zLeg[2],uRightTarget[4],vel_lift , t_diff )
  end

  moveleg.store_stance(t,0,uLeft,uTorso,uRight,2,uTorso, zLeg[1],zLeg[2])

  mcm.set_stance_bodyHeight(bodyHeight)  
  moveleg.ft_compensate(t_diff)

  delta_legs, angleShift = moveleg.get_leg_compensation_new(supportLeg,0,gyro_rpy, angleShift,t_diff)
  moveleg.set_leg_positions(delta_legs)


  mcm.set_status_uTorsoVel({0,0,0})

  local steprequest = mcm.get_walk_steprequest()    
  if steprequest>0 then return "done_step" end

end -- walk.update

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state
