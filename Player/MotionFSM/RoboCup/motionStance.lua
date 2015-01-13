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

local zShift = vector.new{0.0,0.0}
local zvShift = vector.new{0.0,0.0}
local zShiftTarget = vector.new{0.0,0.0}

local aShiftY = vector.new{0,0,0,0}
local avShiftY = vector.new{0.0,0.0}
local aShiftTargetY = vector.new{0.0,0.0}

local aShiftX = vector.new{0,0,0,0}
local avShiftX = vector.new{0.0,0.0}
local aShiftTargetX = vector.new{0.0,0.0}


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
  delta_legs, angleShift = moveleg.get_leg_compensation_simple(supportLeg,0,gyro_rpy, angleShift)


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







--Compensation for arm / objects
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local uTorsoCompensated = util.pose_global(
     {uTorsoComp[1],uTorsoComp[2],0},uTorso)
  mcm.set_stance_bodyHeight(bodyHeight)  











----------------------------------------------------------------------------------------------
-- FT testing
----------------------------------------------------------------------------------------------

if hcm.get_legdebug_enable_balance()>0 then

  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()
  local lf_z,rf_z = l_ft[3],r_ft[3]
  local lt_y,rt_y = l_ft[5],r_ft[5] 
  local lt_x,rt_x = l_ft[4],r_ft[4] 
  if IS_WEBOTS then  
    --FOR WEBOTS, ft sensor is rotated
    --FOR webots, torque readings are inverted too    

    lt_y,rt_y = -l_ft[4],-r_ft[5]      
    lt_x,rt_x = -l_ft[5],-r_ft[4] 
  end
  print(string.format("Fz: %d %d  T_p: %d %d T_r: %d %d", lf_z,rf_z,lt_y,rt_y, lt_x,rt_x))
  
  --roll reading: positive for 




  
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



  local z_shift_max = 0.05
  local z_shift_db = 0.01
  local z_vel_max = 0.1 --max 10cm per sec

  local zShiftTargetDiff = (lf_z-rf_z)*k_const_z


  zShiftTarget[1] = math.min(z_shift_max,math.max(-z_shift_max,zShiftTargetDiff))

  if math.abs(zShiftTarget[1])<z_shift_db then zShiftTarget[1]=0 end

  zShiftTarget[2] = -zShiftTarget[1]

  zvShift[1] =  (zShiftTarget[1]-zShift[1])/t_diff
  zvShift[2] =  (zShiftTarget[2]-zShift[2])/t_diff

  zvShift[1]=math.min(z_vel_max,math.max(-z_vel_max,zvShift[1]))
  zvShift[2]=math.min(z_vel_max,math.max(-z_vel_max,zvShift[2]))

  zShift[1] = zShift[1]+zvShift[1]*t_diff
  zShift[2] = zShift[2]+zvShift[2]*t_diff




  zShift[1],zShift[2]=0,0

  print(string.format("Zshift: %.2f %.2f cm",zShift[1]*100,zShift[2]*100))




------------------------------------------------------------------------------------------------


  --Y neutral torque: 0



  local rpy = Body.get_rpy()
  local y_angle_zero = 3*math.pi/180

  



  local k_balancing_y = -10 / (math.pi/180) --10Nm per 1 deg error

  k_balancing_y = -100 / (math.pi/180) --10Nm per 1 deg error
 
  local balancing_db = 1*math.pi/180

  local roll_err = rpy[1]
  local pitch_err = rpy[2]-y_angle_zero  
  local balancing_torque = pitch_err * k_balancing_y
  virtual_torso_angle = hcm.get_legdebug_torso_angle()



  




  print(string.format("angle err: r %.1f p %.1f",roll_err*180/math.pi, pitch_err*180/math.pi))
  if math.abs(pitch_err)<balancing_db then balancing_torque = 0 end


  --make it conform to external torque

  local k_const_ty =  1 *   math.pi/180   --Y angular spring constant: 10 deg / 5 Nm
  local k_const_tx =  1 *   math.pi/180   --Y angular spring constant: 10 deg / 5 Nm

--for actual robot (inverted torque)
   


  local r_const_ty =   0 --zero damping for now
  local r_const_tx =   0 --zero damping for now  

  local ay_shift_max = 30*math.pi/180
  local ay_shift_db = 1*math.pi/180
  local ay_vel_max = 30*math.pi/180 

  local ax_shift_max = 30*math.pi/180
  local ax_shift_db = 1*math.pi/180
  local ax_vel_max = 30*math.pi/180 


--lt_y,rt_y = 0,0 --zero sensed torque


--  if math.abs(pitch_err)>=balancing_db then

if false then
    --has angle error, stop adjusting ankle


    local vt_angle_vel_limit = 90*math.pi/180
    local vt_angle_limit = 20*math.pi/180

    local k_vt_angle = -2

    local vt_angle_diff = 
      math.min(vt_angle_vel_limit ,  math.max(-vt_angle_vel_limit,  k_vt_angle*pitch_err))
    
    virtual_torso_angle[1] = virtual_torso_angle[1] + vt_angle_diff*t_diff

    virtual_torso_angle[1] = 
      math.min(vt_angle_limit ,  math.max(-vt_angle_limit,  virtual_torso_angle[1]))
     



--    virtual_torso_angle[1] =  - pitch_err
    print("ankleerror vta:",virtual_torso_angle[1]*180/math.pi)

    hcm.set_legdebug_torso_angle(virtual_torso_angle)

  else
    virtual_torso_angle[1]=0


    local aShiftModY1=(lt_y)*k_const_ty + avShiftY[1]*r_const_ty
    local aShiftModY2=(rt_y)*k_const_ty + avShiftY[2]*r_const_ty

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

    print(string.format("Ashift: %.1f %.1f deg",aShiftY[1]*180/math.pi,aShiftY[2]*180/math.pi))   

  end

--  aShiftY[1], aShiftY[2] = 0,0




  --Always adapt ankle roll angles (as they are spreaded)

  local aShiftModX1=(lt_x)*k_const_tx + avShiftX[1]*r_const_tx
  local aShiftModX2=(rt_x)*k_const_tx + avShiftX[2]*r_const_tx

  if math.abs(aShiftModX1)<ax_shift_db then aShiftModX1=0 end
  if math.abs(aShiftModX2)<ax_shift_db then aShiftModX2=0 end

  aShiftTargetX[1] = aShiftX[1] + aShiftModX1
  aShiftTargetX[2] = aShiftX[2] + aShiftModX2

  aShiftTargetX[1] = math.min(ax_shift_max,math.max(-ax_shift_max,aShiftTargetX[1]))
  aShiftTargetX[2] = math.min(ax_shift_max,math.max(-ax_shift_max,aShiftTargetX[2]))

  avShiftX[1] =  (aShiftTargetX[1]-aShiftX[1])/t_diff
  avShiftX[2] =  (aShiftTargetX[2]-aShiftX[2])/t_diff

  avShiftX[1]=math.min(ax_vel_max,math.max(-ax_vel_max,avShiftX[1]))
  avShiftX[2]=math.min(ax_vel_max,math.max(-ax_vel_max,avShiftX[2]))

  aShiftX[1] = aShiftX[1]+avShiftX[1]*t_diff
  aShiftX[2] = aShiftX[2]+avShiftX[2]*t_diff


print(string.format("Rshift: %.1f %.1f deg",aShiftX[1]*180/math.pi,aShiftX[2]*180/math.pi))   

end


  delta_legs[5]=delta_legs[5]+aShiftY[1]
  delta_legs[11]=delta_legs[11]+aShiftY[2]

  delta_legs[6]=delta_legs[6]+aShiftX[1]
  delta_legs[12]=delta_legs[12]+aShiftX[2]





--  moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight,  zShift[1],zShift[2],delta_legs)
  moveleg.set_leg_positions_torsoflex(    
    uTorsoCompensated,uLeft,uRight,  
    zShift[1],zShift[2],delta_legs,
    virtual_torso_angle
    )
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
