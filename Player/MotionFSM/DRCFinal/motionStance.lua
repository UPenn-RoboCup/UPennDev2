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


local virtual_torso_angle = {0,0}

---------------------------
-- State machine methods --
---------------------------

local ankleTilt = vector.new{0,0}

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


  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()  
  hcm.set_legdebug_left({uLeft[1],uLeft[2],uLeft[3],0})
  hcm.set_legdebug_right({uRight[1],uRight[2],uRight[3],0})
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



------------------------------------------------
--External control
  local uLeftTarget = hcm.get_legdebug_left() 
  local uRightTarget = hcm.get_legdebug_right()  
  local uTorsoTarget = hcm.get_legdebug_torso()  

  uLeft[1],uLeft[2],uLeft[3]=
    uLeftTarget[1],uLeftTarget[2],uLeftTarget[3]
  uRight[1],uRight[2],uRight[3]=
    uRightTarget[1],uRightTarget[2],uRightTarget[3]
  uTorso[1],uTorso[2]=
    uTorsoTarget[1],uTorsoTarget[2]
------------------------------------------------

  mcm.set_status_uLeft(uLeft)
  mcm.set_status_uRight(uRight)
  mcm.set_status_uTorso(uTorso)  







--Compensation for arm / objects
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local uTorsoCompensated = util.pose_global(
     {uTorsoComp[1],uTorsoComp[2],0},uTorso)
  mcm.set_stance_bodyHeight(bodyHeight)  






  moveleg.ft_compensate(t_diff)






----------------------------------------------------------------------------------------------
-- FT testing
----------------------------------------------------------------------------------------------




local z_force_zero = 300 --Z Neutral force: 300N
local k_const_z = 0.00005  --Z spring constnat: 0.015m / 300N
--local r_const_z = -0.00000005
local r_const_z = -0.0000 --Let's assume no damping 
local z_shift_max = 0.03

--higher value for robot
k_const_z = 0.0001  --Z spring constnat: 0.015m / 300N
z_shift_max = 0.05


local z_shift_db = 0.005
local z_vel_max = 0.03 --max 5cm per sec


local balancing_db = 1*math.pi/180

--[[
  zShiftTarget[1] = (lf_z-z_force_zero)*k_const_z + zvShift[1]*r_const_z 
  zShiftTarget[2] = (rf_z-z_force_zero)*k_const_z + zvShift[2]*r_const_z

  zShiftTarget[1] = math.min(z_shift_max,math.max(-z_shift_max,zShiftTarget[1]))
  zShiftTarget[2] = math.min(z_shift_max,math.max(-z_shift_max,zShiftTarget[2]))

  if math.abs(zShiftTarget[1])<z_shift_db then zShiftTarget[1]=0 end
  if math.abs(zShiftTarget[2])<z_shift_db then zShiftTarget[2]=0 end

  zvShift[1] =  (zShiftTarget[1]-zShift[1])/t_diff
  zvShift[2] =  (zShiftTarget[2]-zShift[2])/t_diff

  zvShift[1]=math.min(z_vel_max,math.max(-z_vel_max,zvShift[1]))
  zvShift[2]=math.min(z_vel_max,math.max(-z_vel_max,zvShift[2]))

  zShift[1] = zShift[1]+zvShift[1]*t_diff
  zShift[2] = zShift[2]+zvShift[2]*t_diff
--]]





--adapt ankle pitch angles
----------------------------------------------------------------------------------------

--[[

  local adaptive_torque_max = 10
  local balancing_torque = pitch_err/(3*math.pi/180) * 0.5
  local ay_vel_max = 30*math.pi/180 
  
  local ay_vel_max = 90*math.pi/180 

--Check pitch controllability with height modification
  local LR_spread_angle = math.abs((uLeftTorso[1]-uRightTorso[1])/(uLeftTorso[2]-uRightTorso[2]))





  if math.abs(pitch_err)<2*math.pi/180 then

    lt_y = math.min(adaptive_torque_max,math.max(-adaptive_torque_max,lt_y))
    rt_y = math.min(adaptive_torque_max,math.max(-adaptive_torque_max,rt_y))

    local aShiftModY1=(lt_y+balancing_torque)*k_const_ty + avShiftY[1]*r_const_ty
    local aShiftModY2=(rt_y+balancing_torque)*k_const_ty + avShiftY[2]*r_const_ty

    if math.abs(aShiftModY1)<ay_shift_db then aShiftModY1=0 end
    if math.abs(aShiftModY2)<ay_shift_db then aShiftModY2=0 end

    aShiftTargetY[1] = aShiftY[1]+aShiftModY1
    aShiftTargetY[2] = aShiftY[2]+aShiftModY2

    aShiftTargetY[1] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftTargetY[1]))
    aShiftTargetY[2] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftTargetY[2]))
    
    avShiftY[1] =  (aShiftTargetY[1]-aShiftY[1])/t_diff
    avShiftY[2] =  (aShiftTargetY[2]-aShiftY[2])/t_diff

    avShiftY[1]=math.min(ay_vel_max,math.max(-ay_vel_max,avShiftY[1]))
    avShiftY[2]=math.min(ay_vel_max,math.max(-ay_vel_max,avShiftY[2]))

    aShiftY[1] = aShiftY[1]+avShiftY[1]*t_diff
    aShiftY[2] = aShiftY[2]+avShiftY[2]*t_diff
  else

    local aShiftModY1=pitch_err
    local aShiftModY2=pitch_err

    if math.abs(aShiftModY1)<ay_shift_db then aShiftModY1=0 end
    if math.abs(aShiftModY2)<ay_shift_db then aShiftModY2=0 end

    aShiftTargetY[1] = aShiftY[1]+aShiftModY1
    aShiftTargetY[2] = aShiftY[2]+aShiftModY2

    aShiftTargetY[1] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftTargetY[1]))
    aShiftTargetY[2] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftTargetY[2]))
    
    avShiftY[1] =  (aShiftTargetY[1]-aShiftY[1])/t_diff
    avShiftY[2] =  (aShiftTargetY[2]-aShiftY[2])/t_diff

    avShiftY[1]=math.min(ay_vel_max,math.max(-ay_vel_max,avShiftY[1]))
    avShiftY[2]=math.min(ay_vel_max,math.max(-ay_vel_max,avShiftY[2]))

    aShiftY[1] = aShiftY[1]+avShiftY[1]*t_diff
    aShiftY[2] = aShiftY[2]+avShiftY[2]*t_diff

  end

  print(string.format("Ashift: %.1f %.1f deg",aShiftY[1]*180/math.pi,aShiftY[2]*180/math.pi))   

--]]





  


  delta_legs, angleShift = moveleg.get_leg_compensation_simple(supportLeg,0,gyro_rpy, angleShift,t_diff)


--FT adaptation handling





--  moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight,  zShift[1],zShift[2],delta_legs)

  moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight,uLeftTarget[4],uRightTarget[4],delta_legs)
  mcm.set_status_uTorsoVel({0,0,0})

  local steprequest = mcm.get_walk_steprequest()    
  if steprequest>0 then return "done_step" end

  --todo... we can handle body height change here
  mcm.set_status_zLeg({0,0})
end -- walk.update

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state
