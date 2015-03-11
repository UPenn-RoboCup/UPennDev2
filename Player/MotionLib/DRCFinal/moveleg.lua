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
local RAD_TO_DEG = 1/DEG_TO_RAD
local sformat = string.format

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


local queue_count, queue_size = 1,5
local lf_queue = vector.zeros(queue_size)
local rf_queue = vector.zeros(queue_size)
local ltx_queue = vector.zeros(queue_size)
local rtx_queue = vector.zeros(queue_size)
local lty_queue = vector.zeros(queue_size)
local rty_queue = vector.zeros(queue_size)



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
    lf_x=l_ft[1],rf_x=r_ft[1],
    lf_y=l_ft[2],rf_y=r_ft[2],
    lf_z=l_ft[3],rf_z=r_ft[3],
    lt_x=-l_ft[4],rt_x=-r_ft[4],
    lt_y=l_ft[5],rt_y=r_ft[5],
    lt_z=0, rt_z=0 --do we ever need yaw torque?
  }
  if IS_WEBOTS then
    ft.lt_y, ft.rt_y = -l_ft[4],-r_ft[5]      
    ft.lt_x,ft.rt_x = -l_ft[5],-r_ft[4] 
  end
  local rpy = Body.get_rpy()
  local gyro, gyro_t = Body.get_gyro()
  local imu={
    roll_err = rpy[1], pitch_err = rpy[2]-y_angle_zero,  
    v_roll = gyro[1], v_pitch = gyro[2]
  }


  --moving window 
  lf_queue[queue_count] = math.sqrt(ft.lf_z^2+ft.lf_y^2+ft.lf_x^2)
  rf_queue[queue_count] = math.sqrt(ft.rf_z^2+ft.rf_y^2+ft.rf_x^2)
  ltx_queue[queue_count] = ft.lt_x
  rtx_queue[queue_count] = ft.rt_x
  lty_queue[queue_count] = ft.lt_y
  rty_queue[queue_count] = ft.rt_y
  queue_count = queue_count+1
  if queue_count>queue_size then queue_count=1 end
  mcm.set_status_LFT({
    vector.sum(lf_queue)/queue_size,    
    vector.sum(ltx_queue)/queue_size,    
    vector.sum(lty_queue)/queue_size
    })
  mcm.set_status_RFT({
    vector.sum(rf_queue)/queue_size,    
    vector.sum(rtx_queue)/queue_size,    
    vector.sum(rty_queue)/queue_size
    })


  mcm.set_status_IMU({imu.roll_err, imu.pitch_err, v_roll,v_pitch})

  local zf_touchdown = 50
  if IS_WEBOTS then zf_touchdown = 1 end




  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()  
  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso)

  local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeft)
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight)  
  local uTorsoNeutral = util.se2_interpolate(0.5,uLeftSupport, uRightSupport)

  local zmp_err_left = {0,0,0}
  local zmp_err_right = {0,0,0}
  local forceLeft, forceRight = 0,0

  local uZMP = mcm.get_status_uZMP()  
  local uZMPLeft=mcm.get_status_uLeft()
  local uZMPRight=mcm.get_status_uRight()

  if ft.lf_z>zf_touchdown then
    zmp_err_left = {-ft.lt_y/ft.lf_z, ft.lt_x/ft.lf_z, 0}
    uZMPLeft = util.pose_global(zmp_err_left, uLeft)
    forceLeft = ft.lf_z
  end
  if ft.rf_z>zf_touchdown then
    zmp_err_right = {-ft.rt_y/ft.rf_z, ft.rt_x/ft.rf_z, 0}
    uZMPRight = util.pose_global(zmp_err_right, uRight)
    forceRight = ft.rf_z
  end
  local uZMPMeasured= (forceLeft*uZMPLeft + forceRight*uZMPRight) / (forceLeft+forceRight)

  mcm.set_status_LZMP({zmp_err_left[1],zmp_err_left[2],0})
  mcm.set_status_RZMP({zmp_err_right[1],zmp_err_right[2],0})  
  mcm.set_status_uZMPMeasured(uZMPMeasured) 
  mcm.set_status_uTorsoNeutral(uTorsoNeutral)

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
  local ankleShiftXTarget = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftYTarget = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])
  local kneeShiftXTarget = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  local hipShiftYTarget=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])

--[[
  local angleShift = mcm.get_walk_angleShift()

  angleShift[1] = util.p_feedback(angleShift[1], ankleShiftXTarget, ankleImuParamX[1],dShift[1], dt)
  angleShift[2] = util.p_feedback(angleShift[2], ankleShiftYTarget, ankleImuParamY[1], dShift[2], dt)
  angleShift[3] = util.p_feedback(angleShift[3], kneeShiftXTarget, kneeImuParamX[1], dShift[3], dt)
  angleShift[4] = util.p_feedback(angleShift[4], hipShiftYTarget, hipImuParamY[1], dShift[4], dt)
--]]




  local dShiftTarget = {}
  dShiftTarget[1]=ankleImuParamX[1]*(ankleShiftXTarget-angleShift[1])
  dShiftTarget[2]=ankleImuParamY[1]*(ankleShiftYTarget-angleShift[2])
  dShiftTarget[3]=kneeImuParamX[1]*(kneeShiftXTarget-angleShift[3])
  dShiftTarget[4]=hipImuParamY[1]*(hipShiftYTarget-angleShift[4])
  

  angleShift[1] = angleShift[1] + math.max(-dShift[1]*dt,math.min(dShift[1]*dt,dShiftTarget[1]))
  angleShift[2] = angleShift[2] + math.max(-dShift[2]*dt,math.min(dShift[2]*dt,dShiftTarget[2])) 
  angleShift[3] = angleShift[3] + math.max(-dShift[3]*dt,math.min(dShift[3]*dt,dShiftTarget[3])) 
  angleShift[4] = angleShift[4] + math.max(-dShift[4]*dt,math.min(dShift[4]*dt,dShiftTarget[4])) 


  mcm.set_walk_angleShift(angleShift)

  local delta_legs = vector.zeros(12)

  local uTorso = mcm.get_status_uTorso()
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uZMP = mcm.get_status_uZMP()

  local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeft)
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight)

  local dTL = math.sqrt( (uTorso[1]-uLeftSupport[1])^2+ (uTorso[2]-uLeftSupport[2])^2)
  local dTR = math.sqrt((uTorso[1]-uRightSupport[1])^2+(uTorso[2]-uRightSupport[2])^2)
  local supportRatio = math.max(dTL,dTR)/(dTL+dTR)

  --SJ: now we apply the compensation during DS too
  local phComp1 = Config.walk.phComp[1]
  local phComp2 = Config.walk.phComp[2]
  local phCompSlope = Config.walk.phCompSlope

  local phSingleComp = math.min( math.max(ph-phComp1, 0)/(phComp2-phComp1), 1)
  local phComp = math.min( phSingleComp/phCompSlope, 1,
                          (1-phSingleComp)/phCompSlope)
  supportRatioLeft, supportRatioRight = 0,0


  local phComp2 = math.max(0, math.min(1, (supportRatio-0.58)/ (0.66-0.58)) )
  local phCompLift = math.max(0, math.min(1, (supportRatio-0.6)/ (0.94-0.58)) )


  local phCompLift = phComp2 --knee compensation for standard walking too



--  if mcm.get_stance_singlesupport()==1 then phComp = phComp*2 end

	phComp = 0 
  local swing_leg_sag_compensation_left = Config.walk.footSagCompensation[1]
  local swing_leg_sag_compensation_right = Config.walk.footSagCompensation[2]

	local kneeComp={0,0}	
	local knee_compensation = Config.walk.kneePitchCompensation

	local zLegComp = mcm.get_status_zLegComp()

  if dTL>dTR then --Right support
    supportRatioRight = math.max(phComp,phComp2);
		kneeComp[2] = phCompLift*knee_compensation
    mcm.set_walk_zSag({phCompLift*swing_leg_sag_compensation_left,0})

		zLegComp[1] = math.max(-supportRatioRight,zLegComp[1])
  else
    supportRatioLeft = math.max(phComp,phComp2);
		kneeComp[1] = phCompLift*knee_compensation

    mcm.set_walk_zSag({0,phCompLift*swing_leg_sag_compensation_right})
		zLegComp[2] = math.max(-supportRatioLeft,zLegComp[2])
  end


  local lft = mcm.get_status_LFT()
  local rft = mcm.get_status_RFT()
  local afloat_threshold = 50
  local shiftL,shiftR=1,1

  if Config.walk.force_torque then

  if lft[1]< afloat_threshold then --left foot afloat
    shiftL=0
  end
  if rft[1]< afloat_threshold then --left foot afloat
    shiftR=0
  end
  end



  delta_legs[2] = angleShift[4] + hipRollCompensation*supportRatioLeft
  delta_legs[3] = - hipPitchCompensation*supportRatioLeft
  delta_legs[4] = angleShift[3]*shiftL - kneePitchCompensation*supportRatioLeft-kneeComp[1]
  delta_legs[5] = angleShift[1]*shiftL - anklePitchCompensation*supportRatioLeft
  delta_legs[6] = angleShift[2]*shiftL + ankleRollCompensation*supportRatioLeft

  delta_legs[8]  = angleShift[4] - hipRollCompensation*supportRatioRight
  delta_legs[9] = -hipPitchCompensation*supportRatioRight
  delta_legs[10] = angleShift[3]*shiftR - kneePitchCompensation*supportRatioRight-kneeComp[2]
  delta_legs[11] = angleShift[1]*shiftR - anklePitchCompensation*supportRatioRight
  delta_legs[12] = angleShift[2]*shiftR - ankleRollCompensation

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
  zHeight0, stepHeight,zHeight1 = 0, walkParam[2],walkParam[3]
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

  local lift_phase, land_phase = 0,0


  if tf < lift/total_dist then 
    xf,zf =0,tf*total_dist +zHeight0
    lift_phase = 1 - tf / (lift/total_dist) --1 to 0




  elseif tf <(lift+move)/total_dist then 
    xf,zf = (tf*total_dist-lift)/move, stepHeight

  else xf,zf= 1,(1-tf)*total_dist +zHeight1   
    land_phase = (tf*total_dist - (lift+move))/land
  end   
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)

  return uFoot, zf, lift_phase, land_phase
end





function moveleg.set_leg_positions()

  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local uTorsoZMPComp = mcm.get_status_uTorsoZMPComp()

  uTorso = util.pose_global(uTorsoZMPComp,uTorso)

  local zLeg = mcm.get_status_zLeg()
  local zSag = mcm.get_walk_zSag()
    
	local zLegComp = mcm.get_status_zLegComp()

  local zLeft,zRight = zLeg[1]+zSag[1]+zLegComp[1],zLeg[2]+zSag[2]+zLegComp[2]
  local supportLeg = mcm.get_status_supportLeg()
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local uTorsoCompensated = util.pose_global({uTorsoComp[1],uTorsoComp[2],0},uTorso)

  
  local delta_legs = mcm.get_walk_delta_legs()  


  local aShiftX = mcm.get_walk_aShiftX()
  local aShiftY = mcm.get_walk_aShiftY()


--ankle height compensation
local ankle_height = 0.118


local uLeftComp=util.pose_global( {math.sin(aShiftY[1])*ankle_height,0,0},uLeft)
local uRightComp=util.pose_global( {math.sin(aShiftY[2])*ankle_height,0,0},uRight)
local zLeftComp = (1-math.cos(aShiftY[1]))*ankle_height
local zRightComp = (1-math.cos(aShiftY[2]))*ankle_height

--
zLeftComp,zRightComp = 0,0
uLeftComp,uRightComp=uLeft,uRight
--

--  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
--  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})

  local pLLeg = vector.new({uLeftComp[1],uLeftComp[2],zLeft-zLeftComp,0,0,uLeft[3]})
  local pRLeg = vector.new({uRightComp[1],uRightComp[2],zRight-zRightComp,0,0,uRight[3]})


  local qWaist = Body.get_waist_command_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()


  local count,revise_max = 1,4
  local adapt_factor = 1.0

  --Initial guess 
  local uTorsoAdapt = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
    uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
            0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})

  
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso,aShiftX,aShiftY)

  -------------------Incremental COM filtering
  local com_z = 0
  while count<=revise_max do
    local qLLeg = vector.slice(qLegs,1,6)
    local qRLeg = vector.slice(qLegs,7,12)
    com = K.calculate_com_pos(qWaist,qLArm,qRArm,qLLeg,qRLeg,0,0,0, 1,1)
    local uCOM = util.pose_global(
      vector.new({com[1]/com[4], com[2]/com[4],0}),uTorsoAdapt)

   uTorsoAdapt[1] = uTorsoAdapt[1]+ adapt_factor * (uTorso[1]-uCOM[1])
   uTorsoAdapt[2] = uTorsoAdapt[2]+ adapt_factor * (uTorso[2]-uCOM[2])
   local pTorso = vector.new({
            uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
            0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})

   qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, aShiftX, aShiftY)
   count = count+1
  end
  local uTorsoOffset = util.pose_relative(uTorsoAdapt, uTorso)
  
--  print("uTorsoZ:",com[3]/com[4])
--  print("uTorso:",uTorso[1],uLeft[1])
--  print("Torso offset:",uTorsoOffset[1],uTorsoOffset[2])
  mcm.set_stance_COMoffset({
    -uTorsoOffset[1],-uTorsoOffset[2],com[3]/com[4]
    })


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
--  moveleg.process_ft_height(ft,imu,t_diff) -- height adaptation
--  moveleg.process_ft_roll(ft,t_diff) -- roll adaptation
--  moveleg.process_ft_pitch(ft,t_diff) -- pitch adaptation
end






function moveleg.process_ft_height(ft,imu,t_diff)
  --------------------------------------------------------------------------------------------------------
  -- Foot height differential adaptation

  local zvShift={0,0}

  --Torso movement for zmp-based balancing
  local zmp_err_db = {0.01  ,0.0025}  
  local k_zmp_err = -0.25 --0.5cm per sec for 1cm error
  local max_torso_vel = {0.0,0.01} --1cm per sec
  local max_zmp_comp = 0.04


  --Touchdown detection 
  local zf_touchdown = 50
  local zf_touchdown2 = 100
  local zf_support = 200

  local foot_z_vel = -0.02
--  local foot_z_vel = -0.03

  if IS_WEBOTS then
    foot_z_vel = -0.02

    zf_support = 130 --for webots, total mass is ~260N    

    zf_touchdown = 20    
    zf_touchdown2= 90


    zmp_err_db = {0.0025  ,0.0025}  
    max_torso_vel = {0.01,0.01} --1cm per sec

    max_zmp_comp = 0.08


    zf_touchdown = 20    
    zf_touchdown2= 50





  end

  local lft = mcm.get_status_LFT()
  local rft = mcm.get_status_RFT()

  local enable_balance = hcm.get_legdebug_enable_balance()
  local zmp_err_left = mcm.get_status_LZMP()
  local zmp_err_right = mcm.get_status_RZMP()
  local uTorsoZMPComp = mcm.get_status_uTorsoZMPComp()


  local lf_z,rf_z = lft[1],rft[1]
  local lt_y,rt_y = lft[3],rft[3]




	if (lf_z>zf_support and rf_z<zf_touchdown) or enable_balance[2]>0 then
      local torso_x_comp = util.procFunc((zmp_err_left[1]-supportX)*k_zmp_err,zmp_err_db[1],max_torso_vel[1])
      local torso_y_comp = util.procFunc(zmp_err_left[2]*k_zmp_err,zmp_err_db[2],max_torso_vel[2])
      uTorsoZMPComp[1] = util.procFunc(uTorsoZMPComp[1]+ torso_x_comp*t_diff,0,max_zmp_comp)
      uTorsoZMPComp[2] = util.procFunc(uTorsoZMPComp[2]+ torso_y_comp*t_diff,0,max_zmp_comp)
      mcm.set_status_uTorsoZMPComp(uTorsoZMPComp)

  elseif  (rf_z>zf_support and lf_z<zf_touchdown) or enable_balance[1]>0 then
      local torso_x_comp = util.procFunc((zmp_err_right[1]-supportX)*k_zmp_err,zmp_err_db[1],max_torso_vel[1])
      local torso_y_comp = util.procFunc(zmp_err_right[2]*k_zmp_err,zmp_err_db[2],max_torso_vel[2])
      uTorsoZMPComp[1] = util.procFunc(uTorsoZMPComp[1]+ torso_x_comp*t_diff,0,max_zmp_comp)
      uTorsoZMPComp[2] = util.procFunc(uTorsoZMPComp[2]+ torso_y_comp*t_diff,0,max_zmp_comp)
      mcm.set_status_uTorsoZMPComp(uTorsoZMPComp)
  else --Both feet on the ground, so balance them
--    mcm.set_status_uTorsoNeutral(uTorsoNeutral)
    local k_heightbalance = 1/50 * 0.01
    local k_heightreturn = -1

    local foot_z_vel_balance = 
      util.procFunc((lf_z - rf_z) *k_heightbalance, 0.01, 0.02)

    local zShift = mcm.get_status_zLeg()    
    local zAvg = (zShift[1]+zShift[2])/2


    foot_z_vel_balance = 0

    local foot_z_vel_height =0




    if foot_z_vel_balance==0 then 
      if zAvg>0.02 then foot_z_vel_height = -0.01
      elseif zAvg<-0.02 then foot_z_vel_height = 0.01
      end
    end

  

    zvShift[1] = foot_z_vel_balance+foot_z_vel_height
    zvShift[2] = -foot_z_vel_balance+foot_z_vel_height
  end

  local twist_db = 0.2



  if lf_z>zf_support  then --left support

    if enable_balance[2]>0 then --left support  
      if rf_z<zf_touchdown2 and uTorsoZMPComp[2]>-0.02 then
          zvShift[2] = foot_z_vel --Lower left feet 
      end

      if rf_z>zf_touchdown2 and uTorsoZMPComp[2]<-0.02 then
        print("Touchdown detected")
        enable_balance[2]=0
        hcm.set_legdebug_enable_balance(enable_balance)
        hcm.set_state_proceed(1) --auto advance!
      end
    end

  elseif rf_z>zf_support  then  --right support

    if enable_balance[1]>0 then --right support
      if lf_z<zf_touchdown2 and uTorsoZMPComp[2]<0.02 then
        zvShift[1] = foot_z_vel --Lower left feet 
      end
      if lf_z>zf_touchdown2 and uTorsoZMPComp[2]>0.02 then
        print("Touchdown detected")
        enable_balance[1]=0
        hcm.set_legdebug_enable_balance(enable_balance)
        hcm.set_state_proceed(1) --auto advance!
      end
    end
  end



  local zShift = mcm.get_status_zLeg()
  local z_min = -0.05

  zShift[1] = zShift[1]+zvShift[1]*t_diff
  zShift[2] = zShift[2]+zvShift[2]*t_diff

--[[
  zShift[1] = math.max(z_min, zShift[1]+zvShift[1]*t_diff)
  zShift[2] = math.max(z_min, zShift[2]+zvShift[2]*t_diff)
--]]
  mcm.set_walk_zvShift(zvShift)
  mcm.set_walk_zShift(zShift)
  mcm.set_status_zLeg(zShift)
  
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


  if IS_WEBOTS and false then
    k_const_tx = k_const_tx*3
    ax_vel_max = ax_vel_max*3
  end


  local df_max = 100 --full damping beyond this
  local df_min = 30 -- zero damping 

  ----------------------------------------------------------------------------------------
  -- Ankle roll adaptation 

  local aShiftX=mcm.get_walk_aShiftX()
  local avShiftX=mcm.get_walk_avShiftX()

  local enable_balance = hcm.get_legdebug_enable_balance()

  local left_damping_factor = math.max(0,math.min(1, (ft.lf_z-df_min)/df_max))
  local right_damping_factor = math.max(0,math.min(1, (ft.rf_z-df_min)/df_max))



  avShiftX[1] = util.procFunc( ft.lt_x*k_const_tx + 
    avShiftX[1]*r_const_tx*left_damping_factor    
      ,k_const_tx*ax_shift_db*left_damping_factor, 
      ax_vel_max)

  avShiftX[2] = util.procFunc( ft.rt_x*k_const_tx + 
    avShiftX[2]*r_const_tx*right_damping_factor 
      ,k_const_tx*ax_shift_db*right_damping_factor, 
      ax_vel_max)

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
  

  ----------------------------------------------------------------------------------------

end

function moveleg.process_ft_pitch(ft,t_diff)

  local k_const_ty =  20 *   math.pi/180 /5  --Y angular spring constant: 20 deg/s  / 5 Nm
  local r_const_ty =   -0.2 --zero damping for now
  local ay_shift_max = 30*math.pi/180
  local ay_shift_db = 1
  local ay_vel_max = 10*math.pi/180 

  local df_max = 100 --full damping beyond this
  local df_min = 30 -- zero damping 
  local ay_offset = 0


  if IS_WEBOTS then
    --for webots, the torque range is much smaller
    k_const_ty =  20 *   math.pi/180 / 1
    r_const_ty = 0 --no damping for webots  
    ay_shift_db = 0.05
    ay_offset = 0.11         
  end

  ----------------------------------------------------------------------------------------
  -- Ankle pitch adaptation 
  local aShiftY=mcm.get_walk_aShiftY()
  local avShiftY=mcm.get_walk_avShiftY()

  local enable_balance = hcm.get_legdebug_enable_balance()


  local left_damping_factor = math.max(0,math.min(1, (ft.lf_z-df_min)/df_max))
  local right_damping_factor = math.max(0,math.min(1, (ft.rf_z-df_min)/df_max))
    
  avShiftY[1] = util.procFunc(  (ft.lt_y-ay_offset)*k_const_ty + 
    avShiftY[1]*r_const_ty*left_damping_factor 
      ,k_const_ty*ay_shift_db, 
      ay_vel_max)
  avShiftY[2] = util.procFunc(  (ft.rt_y-ay_offset)*k_const_ty + 
    avShiftY[2]*r_const_ty*right_damping_factor    
      ,k_const_ty*ay_shift_db, 
      ay_vel_max)

	--if foot is firmly on the ground, lower the gain a lot (to reduce vibration)
	if ft.lf_z>100 then avShiftY[1] = avShiftY[1]*0.25  end
	if ft.rf_z>100 then avShiftY[2] = avShiftY[2]*0.25  end

--[[
  --foot idle, return to heel strike position
  if ft.lf_z<50 and math.abs(ft.lt_y)<ay_shift_db then
	  aShiftTarget = -10*math.pi/180		
		avShiftY[1] = util.procFunc( (aShiftTarget-aShiftY[1])*0.5, 0, 5*math.pi/180)
  end

  if ft.rf_z<50 and math.abs(ft.rt_y)<ay_shift_db then
	  aShiftTarget = -10*math.pi/180		
		avShiftY[2] = util.procFunc( (aShiftTarget-aShiftY[2])*0.5, 0, 5*math.pi/180)
  end
--]]

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

  ----------------------------------------------------------------------------------------

end






return moveleg
