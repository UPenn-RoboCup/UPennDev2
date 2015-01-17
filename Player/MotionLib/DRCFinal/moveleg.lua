local moveleg={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

require'mcm'

-- SJ: Shared library for 2D leg trajectory generation
-- So that we can reuse them for different controllers

local footY    = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY
local torsoX    = Config.walk.torsoX

local DEG_TO_RAD = math.pi/180


-- Gyro stabilization parameters
local ankleImuParamX = Config.walk.ankleImuParamX
local ankleImuParamY = Config.walk.ankleImuParamY
local kneeImuParamX  = Config.walk.kneeImuParamX
local hipImuParamY   = Config.walk.hipImuParamY

-- Hip sag compensation parameters
local hipRollCompensation = Config.walk.hipRollCompensation or 0
local ankleRollCompensation = Config.walk.ankleRollCompensation or 0
local anklePitchCompensation = Config.walk.anklePitchCompensation or 0
local kneePitchCompensation = Config.walk.kneePitchCompensation or 0
local hipPitchCompensation = Config.walk.hipPitchCompensation or 0

function moveleg.get_ph_single(ph,phase1,phase2) return math.min(1, math.max(0, (ph-phase1)/(phase2-phase1) ))end

function moveleg.store_stance(t,ph,uLeft,uTorso,uRight,supportLeg,uZMP,zLeft,zRight)

  mcm.set_status_t(t)
  mcm.set_status_ph(ph)
  mcm.set_status_uTorso(uTorso)
  mcm.set_status_uLeft(uLeft)
  mcm.set_status_uRight(uRight)
  mcm.set_status_supportLeg(supportLeg)
  mcm.set_status_uZMP(uZMP)

  if zLeft then mcm.set_status_zLeg({zLeft,zRight}) end 
end

function moveleg.get_ft()
  local y_angle_zero = 3*math.pi/180
  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()  
  local ft= {
    lf_z=l_ft[3],rf_z=r_ft[3],
    lt_x=-l_ft[4],rt_x=-r_ft[4],
    lt_y=l_ft[5],rt_y=r_ft[5]
  }
  if IS_WEBOTS then
    ft.lt_y, ft.rt_y = -l_ft[4],-r_ft[5]      
    ft.lt_x,ft.rt_x = -l_ft[5],-r_ft[4] 
  end
  local rpy = Body.get_rpy()
  local gyro, gyro_t = Body.get_gyro()
  local imu={
    roll_err = rpy[1], pitch_err = rpy[2]-y_angle_zero,  v_roll = gyro[1], v_pitch = gyro[2]
  }
  return ft,imu
end



function moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local body_yaw
  if supportLeg == 0 then  -- Left support
    body_yaw = uLeft[3]  - uTorsoActual[3]
  else
    body_yaw = uRight[3] - uTorsoActual[3]
  end
  -- Ankle stabilization  gyro feedback
  --local imu_roll0, imu_pitch0, imu_yaw0 = unpack(Body.get_sensor_imu())
  --math.sin(imuPitch)*bodyHeight, -math.sin(imuRoll)*bodyHeight
  local gyro, gyro_t = Body.get_gyro()
  local gyro_roll0, gyro_pitch0, gyro_yaw0 = unpack(gyro)
  -- Get effective gyro angle considering body yaw offset
  -- Rotate the Roll and pitch about the intended body yaw
  local gyro_roll  = gyro_roll0  * math.cos(body_yaw) - gyro_pitch0 * math.sin(body_yaw)
  local gyro_pitch = gyro_pitch0 * math.cos(body_yaw) - gyro_roll0  * math.sin(body_yaw)


  -- Give these parameters
  return {gyro_roll, gyro_pitch, gyro_yaw0}
end








function moveleg.get_leg_compensation_new(supportLeg, ph, gyro_rpy,angleShift,supportRatio,dt)

--New compensation code to cancelout backlash on ALL leg joints
  dt= dt or 0.010

  --Now we limit the angular velocity of compensation angles 

  local dShift = {30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD}

  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])
  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])

  local dShiftTarget = {}
  dShiftTarget[1]=ankleImuParamX[1]*(ankleShiftX-angleShift[1])
  dShiftTarget[2]=ankleImuParamY[1]*(ankleShiftY-angleShift[2])
  dShiftTarget[3]=kneeImuParamX[1]*(kneeShiftX-angleShift[3])
  dShiftTarget[4]=hipImuParamY[1]*(hipShiftY-angleShift[4])
  
-- Ankle shift is filtered... thus a global
  angleShift[1] = angleShift[1] + math.max(-dShift[1]*dt,math.min(dShift[1]*dt,dShiftTarget[1]))
  angleShift[2] = angleShift[2] + math.max(-dShift[2]*dt,math.min(dShift[2]*dt,dShiftTarget[2])) 
  angleShift[3] = angleShift[3] + math.max(-dShift[3]*dt,math.min(dShift[3]*dt,dShiftTarget[3])) 
  angleShift[4] = angleShift[4] + math.max(-dShift[4]*dt,math.min(dShift[4]*dt,dShiftTarget[4])) 


  local delta_legs = vector.zeros(12)

  --How much do we need to apply the compensation?
  local supportRatioRight = supportRatio;
  local supportRatioLeft = 1-supportRatio;
--  supportRatioLeft = math.max(0,supportRatioLeft*4-3);
--  supportRatioRight = math.max(0,supportRatioRight*4-3);

  supportRatioLeft = math.max(0,supportRatioLeft*2-1);
  supportRatioRight = math.max(0,supportRatioRight*2-1);



--print("SR:",supportRatio,supportRatioLeft,supportRatioRight)
  --SJ: now we apply the compensation during DS too
  local phComp1 = Config.walk.phComp[1]
  local phComp2 = Config.walk.phComp[2]
  local phCompSlope = Config.walk.phCompSlope

  local phSingleComp = math.min( math.max(ph-phComp1, 0)/(phComp2-phComp1), 1)
  local phComp = math.min( phSingleComp/phCompSlope, 1,
                          (1-phSingleComp)/phCompSlope)
  supportRatioLeft, supportRatioRight = 0,0


  if mcm.get_stance_singlesupport()==1 then
    phComp = phComp*2
  end


  if supportLeg == 0 then -- Left supports
    supportRatioLeft = phComp;
  elseif supportLeg==1 then
    supportRatioRight = phComp;
  end

  delta_legs[2] = angleShift[4] + hipRollCompensation*supportRatioLeft
  delta_legs[3] = - hipPitchCompensation*supportRatioLeft
  delta_legs[4] = angleShift[3] - kneePitchCompensation*supportRatioLeft
  delta_legs[5] = angleShift[1] - anklePitchCompensation*supportRatioLeft
  delta_legs[6] = angleShift[2] + ankleRollCompensation*supportRatioLeft

  delta_legs[8]  = angleShift[4] - hipRollCompensation*supportRatioRight
  delta_legs[9] = -hipPitchCompensation*supportRatioRight
  delta_legs[10] = angleShift[3] - kneePitchCompensation*supportRatioRight
  delta_legs[11] = angleShift[1] - anklePitchCompensation*supportRatioRight
  delta_legs[12] = angleShift[2] - ankleRollCompensation

  mcm.set_walk_delta_legs(delta_legs)  

  return delta_legs, angleShift
end


























function moveleg.foot_trajectory_base(phSingle,uStart,uEnd,stepHeight)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf

  return uFoot, zFoot
end

function moveleg.foot_trajectory_square(phSingle,uStart,uEnd, stepHeight, walkParam)
  local xf,zf,zFoot,aFoot, zHeight0, zHeight1= 0,0,0,0,0,0
  if not walkParam then walkParam={0,stepHeight,0} end
  zHeight0, stepHeight,zHeight1 = walkParam[1],walkParam[2],walkParam[3]
  local lift = math.abs(zHeight0-stepHeight)
  local land = math.abs(zHeight1-stepHeight)
  local move = math.sqrt( (uEnd[2]-uStart[2])*(uEnd[2]-uStart[2])+
                         (uEnd[1]-uStart[1])*(uEnd[1]-uStart[1]))
  local total_dist = lift+land+move

  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local tf = .5*(1-math.cos(math.pi*phSingleSkew))

--cubic curve
  local k =1.5
  if phSingleSkew<0.5 then
    tf = k*phSingleSkew^3 + (2-k/2)*phSingleSkew*phSingleSkew
  else
    tf = 1- (k*(1-phSingleSkew)^3 + (2-k/2)*(1-phSingleSkew)^2)
  end
  tf = math.max(0,math.min(1,tf))

  if tf < lift/total_dist then xf,zf =0,tf*total_dist +zHeight0
  elseif tf <(lift+move)/total_dist then xf,zf = (tf*total_dist-lift)/move, stepHeight
  else xf,zf= 1,(1-tf)*total_dist +zHeight1   end 
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  return uFoot, zf
end





function moveleg.set_leg_positions()


  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local zLeg = mcm.get_status_zLeg()
  local zLeft,zRight = zLeg[1],zLeg[2]
  local supportLeg = mcm.get_status_supportLeg()
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local uTorsoCompensated = util.pose_global({uTorsoComp[1],uTorsoComp[2],0},uTorso)

  local zShift=mcm.get_walk_zShift()
  local aShiftX=mcm.get_walk_aShiftX()
  local aShiftY=mcm.get_walk_aShiftY()
  local delta_legs = mcm.get_walk_delta_legs()  

  zLeft = zLeft + zShift[1]
  zRight = zRight + zShift[2]

  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], mcm.get_stance_bodyHeight(),
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
  
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso)
  local legBias = vector.new(mcm.get_leg_bias())

  qLegs = qLegs + delta_legs + legBias

  qLegs[5]=qLegs[5]+aShiftY[1]
  qLegs[11]=qLegs[11]+aShiftY[2]
  qLegs[6]=qLegs[6]+aShiftX[1]
  qLegs[12]=qLegs[12]+aShiftX[2]

  Body.set_lleg_command_position(vector.slice(qLegs,1,6))
  Body.set_rleg_command_position(vector.slice(qLegs,7,12))

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------
end





function moveleg.ft_compensate(t_diff)

  local enable_balance = hcm.get_legdebug_enable_balance()
  local ft,imu = moveleg.get_ft()
  if enable_balance[1]+enable_balance[2]>0 then
    print()
    print(string.format("%d%d Fz: %d %d  T_p: %d %d T_r: %d %d", 
      enable_balance[1],enable_balance[2],
      ft.lf_z,ft.rf_z,ft.lt_y,ft.rt_y, ft.lt_x,ft.rt_x))
    print(string.format("angle: %.1f p %.1f",imu.roll_err*180/math.pi, imu.pitch_err*180/math.pi))
  end
  moveleg.process_ft_height(ft,imu,t_diff) -- height adaptation
  moveleg.process_ft_roll(ft,t_diff) -- roll adaptation
  moveleg.process_ft_pitch(ft,t_diff) -- pitch adaptation

end






function moveleg.process_ft_height(ft,imu,t_diff)
  --------------------------------------------------------------------------------------------------------
  -- Foot height differential adaptation

  local zf_touchdown = 50
  local z_shift_max = 0.05 --max 5cm difference
  local z_vel_max_diff = 0.4 --max 40cm per sec
  local z_vel_max_balance = 0.05 --max 5cm per sec
  local k_const_z_diff = 0.5 / 100  -- 50cm/s for 100 N difference
  local z_shift_diff_db = 50 --50N deadband
  local k_balancing = 0.4 


  local enable_balance = hcm.get_legdebug_enable_balance()

  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()  

  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso)

  local zvShift={0,0}
  local balancing_type=0


  local enable_adapt = false


  if ((ft.lf_z<zf_touchdown and ft.rf_z>zf_touchdown) or (ft.lf_z>zf_touchdown and ft.rf_z<zf_touchdown) )
    and math.abs(imu.roll_err)<2*math.pi/180
    and enable_adapt

    then 
    

    local zvShiftTarget = util.procFunc( (ft.lf_z-ft.rf_z)*k_const_z_diff , z_shift_diff_db*k_const_z_diff, z_vel_max_diff)
    if enable_balance[1]>0 then zvShift[1] = zvShiftTarget end
    if enable_balance[2]>0 then zvShift[2] = -zvShiftTarget end      
  else
    balancing_type=1
    --both feet on the ground. use IMU to keep torso orientation up

    local LR_pitch_err = -(uLeftTorso[1]-uRightTorso[1])*math.tan(imu.pitch_err)
    local LR_roll_err =  (uLeftTorso[2]-uRightTorso[2])*math.tan(imu.roll_err)
    local zvShiftTarget = util.procFunc( (LR_pitch_err + LR_roll_err) * k_balancing, 0, z_vel_max_balance )

    if enable_balance[1]>0 then zvShift[1] = zvShiftTarget end
    if enable_balance[2]>0 then zvShift[2] = -zvShiftTarget end  
  end

  local zShift = mcm.get_walk_zShift()
  zShift[1] = util.procFunc( zShift[1]+zvShift[1]*t_diff , 0, z_shift_max)
  zShift[2] = util.procFunc( zShift[2]+zvShift[2]*t_diff , 0, z_shift_max)
  mcm.set_walk_zShift(zShift)
  mcm.set_walk_zvShift(zvShift)

  if enable_balance[1]+enable_balance[2]>0 then
    if balancing_type>0 then print"DS balancing" end
    print(string.format("dZ : %.1f Zshift: %.2f %.2f cm",zvShift[1],zShift[1]*100,zShift[2]*100))
  end
  --------------------------------------------------------------------------------------------------------
end

function moveleg.process_ft_roll(ft,t_diff)

--[[
  local k_const_tx =   20 * math.pi/180 /5  --Y angular spring constant: 20 deg/s  / 5 Nm
  local r_const_tx =   0 --zero damping for now  
  local ax_shift_db =  0.3 -- 0.3Nm deadband
  local ax_vel_max = 30*math.pi/180 
  local ax_shift_max = 30*math.pi/180
--]]

--slower, more damped
  local k_const_tx =  10 *   math.pi/180 /5  --Y angular spring constant: 10 deg/s  / 5 Nm
  local r_const_tx =   -0.2 --zero damping for now
  local ax_shift_max = 30*math.pi/180
  local ax_shift_db = 1
  local ax_vel_max = 10*math.pi/180 



  ----------------------------------------------------------------------------------------
  -- Ankle roll adaptation 

  local aShiftX=mcm.get_walk_aShiftX()
  local avShiftX=mcm.get_walk_avShiftX()

  local enable_balance = hcm.get_legdebug_enable_balance()

  avShiftX[1] = util.procFunc( ft.lt_x*k_const_tx + avShiftX[1]*r_const_tx    
      ,k_const_tx*ax_shift_db, ax_vel_max)
  avShiftX[2] = util.procFunc( ft.rt_x*k_const_tx + avShiftX[2]*r_const_tx    
      ,k_const_tx*ax_shift_db, ax_vel_max)

  if enable_balance[1]>0 then
    aShiftX[1] = aShiftX[1]+avShiftX[1]*t_diff
    aShiftX[1] = math.min(ax_shift_max,math.max(-ax_shift_max,aShiftX[1]))
  end

  if enable_balance[2]>0 then
    aShiftX[2] = aShiftX[2]+avShiftX[2]*t_diff
    aShiftX[2] = math.min(ax_shift_max,math.max(-ax_shift_max,aShiftX[2]))
  end

  mcm.set_walk_aShiftX(aShiftX)
  mcm.set_walk_avShiftX(avShiftX)

  if enable_balance[1]+enable_balance[2]>0 then
  print(string.format("dRoll: %.1f %.1f Roll: %.1f %.1f",
    avShiftX[1]*180/math.pi,avShiftX[2]*180/math.pi,
    aShiftX[1]*180/math.pi,aShiftX[2]*180/math.pi))   
  end

  ----------------------------------------------------------------------------------------

end

function moveleg.process_ft_pitch(ft,t_diff)

  local k_const_ty =  10 *   math.pi/180 /5  --Y angular spring constant: 10 deg/s  / 5 Nm
  local r_const_ty =   -0.2 --zero damping for now
  local ay_shift_max = 30*math.pi/180
  local ay_shift_db = 1
  local ay_vel_max = 10*math.pi/180 

  ----------------------------------------------------------------------------------------
  -- Ankle pitch adaptation 
  local aShiftY=mcm.get_walk_aShiftY()
  local avShiftY=mcm.get_walk_avShiftY()

  local enable_balance = hcm.get_legdebug_enable_balance()
    
  avShiftY[1] = util.procFunc(  ft.lt_y*k_const_ty + avShiftY[1]*r_const_ty    
      ,k_const_ty*ay_shift_db, ay_vel_max)
  avShiftY[2] = util.procFunc(  ft.rt_y*k_const_ty + avShiftY[2]*r_const_ty    
      ,k_const_ty*ay_shift_db, ay_vel_max)

  if enable_balance[1]>0 then
    aShiftY[1] = aShiftY[1]+avShiftY[1]*t_diff
    aShiftY[1] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftY[1]))
  end

  if enable_balance[2]>0 then
    aShiftY[2] = aShiftY[2]+avShiftY[2]*t_diff
    aShiftY[2] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftY[2]))
  end

  mcm.set_walk_aShiftY(aShiftY)
  mcm.set_walk_avShiftY(avShiftY)

  if enable_balance[1]+enable_balance[2]>0 then
    print(string.format("dPitch: %.1f %.1f Pitch: %.1f %.1f",
      avShiftY[1]*180/math.pi,avShiftY[2]*180/math.pi,
      aShiftY[1]*180/math.pi,aShiftY[2]*180/math.pi))   
  end
  ----------------------------------------------------------------------------------------

end






return moveleg
