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
local hipRollCompensation =  mcm.get_walk_hipRollCompensation()
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

local function eval_spline(breaks,coefs,ph)
  local x_offset, xf = 0,0
  for i=1,#breaks do
    if ph<=breaks[i] then
      local x=ph - x_offset
      xf = coefs[i][1]*x^3 + coefs[i][2]*x^2 + coefs[i][3]*x + coefs[i][4]
      break;
    end
    x_offset = breaks[i]    
  end
  return xf
end


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
  local y_angle_zero = 0*math.pi/180
  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()  
  local ft= {
    lf_x=l_ft[1],rf_x=r_ft[1],
    lf_y=l_ft[2],rf_y=r_ft[2],
    lf_z=l_ft[3],rf_z=r_ft[3],
    lt_x=l_ft[4],rt_x=r_ft[4],
    lt_y=l_ft[5],rt_y=r_ft[5],
    lt_z=0, rt_z=0 --do we ever need yaw torque?
  }
  if IS_WEBOTS then
    ft.lt_x,ft.rt_x = -l_ft[4],-r_ft[4] 
    ft.lt_y, ft.rt_y = -l_ft[5],-r_ft[5]
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
    zmp_err_left = {-ft.lt_y/ft.lf_z, -ft.lt_x/ft.lf_z, 0}
    uZMPLeft = util.pose_global(zmp_err_left, uLeft)
    forceLeft = ft.lf_z
  end
  if ft.rf_z>zf_touchdown then
    zmp_err_right = {-ft.rt_y/ft.rf_z, -ft.rt_x/ft.rf_z, 0}
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






function moveleg.update_sensory_feedback()
  local y_angle_zero = 0*math.pi/180
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
    ft.lt_x,ft.rt_x = -l_ft[4],-r_ft[4] 
    ft.lt_y, ft.rt_y = -l_ft[5],-r_ft[5]
  end

  local t= Body.get_time()
  local imu_t = Body.get_imu_t()
  local imu_t0 = Body.get_imu_t0()

  local rpy = Body.get_rpy()
  local gyro, gyro_t = Body.get_gyro()
  local imu={rpy[1],rpy[2],rpy[3], gyro[1],gyro[2],gyro[3]}

  if (t-imu_t)>1.0 or (t-imu_t0<2.0) then
    --imu data is old or it is just initialized, ignore gyro data
    imu[1],imu[2],imu[3]=0,0,0
    imu[4],imu[5],imu[6]=0,0,0
  end
  mcm.set_status_IMU(imu)

--  print(string.format("IMU T0:%f sec ago T:%f sec ago",
--    t-imu_t0, t-imu_t ))

  --Filter FT sensor values with moving average
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


  local zf_touchdown = 50
  if IS_WEBOTS then zf_touchdown = 1 end

  --Calculate total zmp position
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

  --just return gyro rpy value (for simple feedback)
  return {imu[4],imu[5],imu[6]}
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


  hipRollCompensation =  mcm.get_walk_hipRollCompensation()

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

  stabilization_mode = mcm.get_status_stabilization_mode()
  if stabilization_mode==2 then kneeShiftXTarget = 0 end --don't use knee stabilization during climbing (stinction possible)

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
	local knee_compensation = Config.walk.kneePitchCompensation or 0

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



  delta_legs[2] = angleShift[4] + hipRollCompensation[1]*supportRatioLeft
  delta_legs[3] = - hipPitchCompensation*supportRatioLeft
  delta_legs[4] = angleShift[3]*shiftL - kneePitchCompensation*supportRatioLeft-kneeComp[1]
  delta_legs[5] = angleShift[1]*shiftL - anklePitchCompensation*supportRatioLeft
  delta_legs[6] = angleShift[2]*shiftL + ankleRollCompensation*supportRatioLeft

  delta_legs[8]  = angleShift[4] - hipRollCompensation[2]*supportRatioRight
  delta_legs[9] = -hipPitchCompensation*supportRatioRight
  delta_legs[10] = angleShift[3]*shiftR - kneePitchCompensation*supportRatioRight-kneeComp[2]
  delta_legs[11] = angleShift[1]*shiftR - anklePitchCompensation*supportRatioRight
  delta_legs[12] = angleShift[2]*shiftR - ankleRollCompensation

  mcm.set_walk_delta_legs(delta_legs)  

  return delta_legs, angleShift
end


function moveleg.get_foot_tilt(ph)
  local aFoot = math.sin(ph*2*math.pi)
  return aFoot
end


function moveleg.foot_trajectory_base(phSingle,uStart,uEnd,stepHeight)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf

  local aFoot = math.sin(phSingle*2*math.pi)

  
  return uFoot, zFoot, aFoot, lift_phase, land_phase
end


function moveleg.foot_trajectory_base2(phSingle,uStart,uEnd,stepHeight)
  --smooth landing, earlierx stop
  local breaksTX={0.150000,0.350000,0.600000,0.800000,1.000000,}
  local breaksTY={0.200000,0.400000,0.500000,0.600000,0.800000,0.900000,1.000000,}
  local coefsX={
    {-4.743550,5.086061,-0.056179,0.000000,},
    {-4.743550,2.951463,1.149449,0.090000,},
    {-2.994272,0.105333,1.760809,0.400000,},
    {0.650617,-2.140370,1.252049,0.800000,},
    {0.650617,-1.750000,0.473975,0.970000,},
  }
  local coefsY={
    {-23.533333,16.495000,-0.457667,0.000000,},
    {-23.533333,2.375000,3.316333,0.380000,},
    {23.216667,-11.745000,1.442333,0.950000,},
    {-31.183333,-4.780000,-0.210167,1.000000,},
    {41.966667,-14.135000,-2.101667,0.900000,},
    {-18.483333,11.045000,-2.719667,0.250000,},
    {-18.483333,5.500000,-1.065167,0.070000,},
  }

  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight*zf
  local aFoot = math.sin(phSingle*2*math.pi) 

  return uFoot, zFoot, aFoot, lift_phase, land_phase
end

function moveleg.foot_trajectory_base3(phSingle,uStart,uEnd,stepHeight)
  --vertical landing 
  local breaksTX={0.200000,0.500000,0.700000,1.000000,}
  local breaksTY={0.200000,0.300000,0.400000,0.600000,0.900000,1.000000,}
  local coefsX={
    {-1.576577,2.770270,0.009009,0.000000,},
    {-1.576577,1.824324,0.927928,0.100000,},
    {-3.198198,0.405405,1.596847,0.500000,},
    {-3.198198,-1.513514,1.375225,0.810000,},
  }
  local coefsY={
    {3.994134,-9.497067,5.489648,0.000000,},
    {3.994134,-7.100587,2.170117,0.750000,},
    {22.041063,-5.902346,0.869824,0.900000,},
    {-6.064527,0.709972,0.350587,0.950000,},
    {-7.720842,-2.928744,-0.093168,1.000000,},
    {-7.720842,-9.877502,-3.935041,0.500000,},
  }
  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight*zf
  local aFoot = math.sin(phSingle*2*math.pi) 

  return uFoot, zFoot, aFoot, lift_phase, land_phase
end



function moveleg.foot_trajectory_soft(phSingle,uStart,uEnd,stepHeight)

--super slow landing for alignment step

  local breaksTX={0.150000,0.300000,0.400000,0.850000,1.000000,}
  local breaksTY={0.100000,0.200000,0.350000,0.450000,0.700000,1.000000,}
  local coefsX={
    {1.505061,4.211612,-0.065606,0.000000,},
    {1.505061,4.888889,1.299469,0.090000,},
    {-42.434434,5.566166,2.867728,0.400000,},
    {5.807140,-7.164164,2.707928,0.700000,},
    {5.807140,0.675475,-0.211982,0.997000,},
  }
  local coefsY={
    {-55.096427,28.528928,-0.501929,0.000000,},
    {-55.096427,12.000000,3.550964,0.180000,},
    {-42.314026,-4.528928,4.298071,0.600000,},
    {77.382769,-23.570240,0.083196,1.000000,},
    {3.171534,-0.355409,-2.309369,0.850000,},
    {3.171534,2.023241,-1.892411,0.300000,},
  }

  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight*zf
  local aFoot = 0
  return uFoot, zFoot, aFoot, lift_phase, land_phase
end



function moveleg.foot_trajectory_softfast(phSingle,uStart,uEnd,stepHeight)

local breaksTX={0.150000,0.300000,0.400000,0.850000,1.000000,}
local breaksTY={0.100000,0.200000,0.350000,0.450000,0.700000,1.000000,}
local coefsX={
  {1.505061,4.211612,-0.065606,0.000000,},
  {1.505061,4.888889,1.299469,0.090000,},
  {-42.434434,5.566166,2.867728,0.400000,},
  {5.807140,-7.164164,2.707928,0.700000,},
  {5.807140,0.675475,-0.211982,0.997000,},
}
local coefsY={
  {-62.248559,28.174568,0.105029,0.000000,},
  {-62.248559,9.500000,3.872486,0.230000,},
  {-8.689349,-9.174568,3.905029,0.650000,},
  {24.234998,-13.084775,0.566127,1.000000,},
  {9.237541,-5.814275,-1.323778,0.950000,},
  {9.237541,1.113881,-2.498876,0.400000,},
}
  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight*zf
  local aFoot = 0
  return uFoot, zFoot, aFoot, lift_phase, land_phase
end


function moveleg.foot_trajectory_walkkick(phSingle,uStart,uEnd,stepHeight)

  local breaksTX={0.300000,0.400000,0.600000,0.800000,0.900000,1.000000,}
  local breaksTY={0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
  local coefsX={
    {8.359213,0.815217,-0.163561,0.000000,},
    {8.359213,8.338509,2.582557,0.250000,},
    {-54.257246,10.846273,4.501035,0.600000,},
    {31.573499,-21.708075,2.328675,1.500000,},
    {34.213251,-2.763975,-2.565735,1.350000,},
    {34.213251,7.500000,-2.092133,1.100000,},
  }
  local coefsY={
    {5.646481,-9.517185,5.346972,0.000000,},
    {5.646481,-4.435352,1.161211,0.900000,},
    {-8.878887,-1.047463,0.064648,1.000000,},
    {5.728314,-6.374795,-1.419804,0.900000,},
    {-1.145663,-4.656301,-2.522913,0.700000,},
    {-1.145663,-5.000000,-3.488543,0.400000,},
  }
  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*1.5
  return uFoot, zFoot,0
end

function moveleg.foot_trajectory_walkkick2(phSingle,uStart,uEnd,stepHeight)

  local breaksTX={0.300000,0.400000,0.600000,0.800000,0.900000,1.000000,}
  local breaksTY={0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
  local breaksA={0.300000,0.500000,0.700000,1.000000,}
  local coefsX={
    {8.359213,0.815217,-0.163561,0.000000,},
    {8.359213,8.338509,2.582557,0.250000,},
    {-54.257246,10.846273,4.501035,0.600000,},
    {31.573499,-21.708075,2.328675,1.500000,},
    {34.213251,-2.763975,-2.565735,1.350000,},
    {34.213251,7.500000,-2.092133,1.100000,},
  }
  local coefsY={
    {5.646481,-9.517185,5.346972,0.000000,},
    {5.646481,-4.435352,1.161211,0.900000,},
    {-8.878887,-1.047463,0.064648,1.000000,},
    {5.728314,-6.374795,-1.419804,0.900000,},
    {-1.145663,-4.656301,-2.522913,0.700000,},
    {-1.145663,-5.000000,-3.488543,0.400000,},
  }
  local coefsA={
    {773.809524,-952.380952,282.738095,0.000000,},
    {773.809524,-255.952381,-79.761905,20.000000,},
    {-59.523810,208.333333,-89.285714,0.000000,},
    {-59.523810,172.619048,-13.095238,-10.000000,},
  }
  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local af=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*1.5
  local aFoot = af*DEG_TO_RAD
  return uFoot, zFoot, aFoot
end




--csapi([0 0.1 0.3 0.7 0.8 0.9 1],[0 -0.2 -1 2  2 1.4 1])
function moveleg.foot_trajectory_kick(phSingle,uStart,uEnd,stepHeight)
    --More swing back

  local breaksTX={0.100000,0.400000,0.670000,0.720000,0.800000,0.900000,1.000000,}
  local breaksTY={0.100000,0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
  local coefsX={
    {103.699049,-45.182858,-3.518705,0.000000,},
    {103.699049,-14.073143,-9.444305,-0.700000,},
    {-229.010397,79.256001,10.110553,-2.000000,},
    {995.160754,-106.242421,2.824219,2.000000,},
    {-485.346581,43.031692,-0.336317,2.000000,},
    {311.504957,-73.451487,-2.769901,2.000000,},
    {311.504957,20.000000,-8.115050,1.300000,},
  }
  local coefsY={
    {-32.433041,1.306550,7.193675,0.000000,},
    {-32.433041,-8.423363,6.481994,0.700000,},
    {108.898830,-27.883187,-0.779316,1.400000,},
    {-165.662278,37.456111,1.135269,1.000000,},
    {295.588566,-61.941256,-3.761760,1.400000,},
    {-39.117713,26.735314,-7.282354,0.700000,},
    {-39.117713,15.000000,-3.108823,0.200000,},
  }
  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*2.5
  return uFoot, zFoot,0
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

  local aFoot = 0
  return uFoot, zf,aFoot, lift_phase, land_phase
end



function moveleg.joint_trajectory_kick(phSingle)


--reference one, >4m
local breaksTX={0.100000,0.300000,0.340000,0.500000,1.000000,}
local breaksTY={0.140000,0.450000,0.600000,0.800000,1.000000,}
local breaksTA={0.120000,0.200000,0.500000,0.750000,1.000000,}
local coefsX={
  {-4719.769765,2558.241239,-203.626426,-16.720000,},
  {-4719.769765,1142.310310,166.428729,-16.220000,},
  {6625.988246,-1689.551549,56.980481,25.000000,},
  {1298.366137,-894.432960,-46.378900,25.000000,},
  {1298.366137,-271.217214,-232.882927,0.000000,},
}
local coefsY={
  {129.678234,-154.897255,57.643922,33.610000,},
  {129.678234,-100.432397,21.897771,39.000000,},
  {-594.422562,20.168361,-2.984080,40.000000,},
  {538.036319,-247.321792,-37.057094,38.000000,},
  {538.036319,75.500000,-71.421453,25.000000,},
}
local coefsA={
  {-2192.431733,1664.494821,-360.751695,-16.890000,},
  {-2192.431733,875.219397,-55.985989,-40.000000,},
  {-703.686964,349.035782,41.954426,-40.000000,},
  {475.043315,-284.282486,61.380414,-15.000000,},
  {475.043315,72.000000,8.309793,-10.000000,},
}
--[[
--toe up, less swing
local breaksTX={0.100000,0.300000,0.340000,0.500000,1.000000,}
local breaksTY={0.140000,0.450000,0.600000,0.800000,1.000000,}
local breaksTA={0.120000,0.200000,0.500000,0.750000,1.000000,}
local coefsX={
  {-4719.769765,2558.241239,-203.626426,-16.720000,},
  {-4719.769765,1142.310310,166.428729,-16.220000,},
  {6625.988246,-1689.551549,56.980481,25.000000,},
  {1298.366137,-894.432960,-46.378900,25.000000,},
  {1298.366137,-271.217214,-232.882927,0.000000,},
}
local coefsY={
  {129.678234,-154.897255,57.643922,33.610000,},
  {129.678234,-100.432397,21.897771,39.000000,},
  {-594.422562,20.168361,-2.984080,40.000000,},
  {538.036319,-247.321792,-37.057094,38.000000,},
  {538.036319,75.500000,-71.421453,25.000000,},
}
local coefsA={
  {-2192.431733,1664.494821,-360.751695,-16.890000,},
  {-2192.431733,875.219397,-55.985989,-40.000000,},
  {-703.686964,349.035782,41.954426,-40.000000,},
  {475.043315,-284.282486,61.380414,-15.000000,},
  {475.043315,72.000000,8.309793,-10.000000,},
}


--take 3
local breaksTX={0.100000,0.300000,0.340000,0.500000,1.000000,}







--]]

  local hip=eval_spline(breaksTX, coefsX,phSingle) *DEG_TO_RAD
  local knee=eval_spline(breaksTY, coefsY,phSingle)  *DEG_TO_RAD
  local ankle=eval_spline(breaksTA, coefsA,phSingle)  *DEG_TO_RAD

  return hip,knee,ankle
end



local debug_out =true
local fout
if debug_out then fout = io.open("webots_debug.txt","w") end
local debug_count=1

function moveleg.set_leg_positions()
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local qWaist = Body.get_waist_command_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  local uTorsoOffset,qLegs,comZ= Body.get_torso_compensation(qLArm,qRArm,qWaist)  
  local delta_legs = mcm.get_walk_delta_legs()  
  mcm.set_stance_COMoffset({-uTorsoOffset[1],-uTorsoOffset[2],comZ })

  --Knee angle check
  if Config.birdwalk then
    --knee pitch should be neagitve
    qLegs[4]= math.min(0,qLegs[4])
    qLegs[10]= math.min(0,qLegs[10])
  else
    --knee pitch should be positive
    qLegs[4]= math.max(0,qLegs[4])
    qLegs[10]= math.max(0,qLegs[10])  
  end

  local legBias = vector.new(mcm.get_leg_bias())
  qLegs = qLegs + delta_legs + legBias
  qLegs[5]=qLegs[5]
  qLegs[11]=qLegs[11]
  qLegs[6]=qLegs[6]
  qLegs[12]=qLegs[12]


  local supportLeg = mcm.get_status_supportLeg()
  


  if Config.walk.anklePitchLimit then
    qLegs[5]=math.max(qLegs[5],Config.walk.anklePitchLimit[1])
    qLegs[11]=math.max(qLegs[11],Config.walk.anklePitchLimit[1])
    qLegs[5]=math.min(qLegs[5],Config.walk.anklePitchLimit[2])
    qLegs[11]=math.min(qLegs[11],Config.walk.anklePitchLimit[2])
  end


  --hip pitch lag fix
  if Config.walk.hipPitch0 then
    if qLegs[3]<Config.walk.hipPitch0 then
      qLegs[3] = Config.walk.hipPitch0 + (qLegs[3]-Config.walk.hipPitch0)*Config.walk.hipPitchCompensationMag
    end
    if qLegs[9]<Config.walk.hipPitch0 then
      qLegs[9] = Config.walk.hipPitch0 + (qLegs[9]-Config.walk.hipPitch0)*Config.walk.hipPitchCompensationMag
    end
  end
  Body.set_lleg_command_position(vector.slice(qLegs,1,6))
  Body.set_rleg_command_position(vector.slice(qLegs,7,12))

if debug_out then
  debug_count=debug_count+1
  if debug_count%10==0 then
    local LLeg=Body.get_lleg_position()
    fout:write(sformat(
    '%.2f    %.2f %.2f %.2f %.2f %.2f %.2f\n',
    Body.get_time(),
    qLegs[3],qLegs[4],qLegs[5],
    LLeg[3],LLeg[4],LLeg[5]
    ))
  fout:flush()
  end
end

--[[
  print(
    string.format("L %.2f R %.2f",
        (qLegs[3]+qLegs[4]+qLegs[5])*180/math.pi,
      (qLegs[9]+qLegs[10]+qLegs[11])*180/math.pi
      )

    )
--]]

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------
  if IS_WEBOTS then
    local llt = Body.get_lleg_current()
    local rlt = Body.get_rleg_current() 

    mcm.set_status_lleg_torque(llt)
    mcm.set_status_rleg_torque(rlt)
  end
end



function moveleg.set_leg_positions_hack(supportLeg,phSingle)
    local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local qWaist = Body.get_waist_command_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  local uTorsoOffset,qLegs,comZ= Body.get_torso_compensation(qLArm,qRArm,qWaist)  
  local delta_legs = mcm.get_walk_delta_legs()  
  mcm.set_stance_COMoffset({-uTorsoOffset[1],-uTorsoOffset[2],comZ })

  --Knee angle check
  if Config.birdwalk then
    --knee pitch should be neagitve
    qLegs[4]= math.min(0,qLegs[4])
    qLegs[10]= math.min(0,qLegs[10])
  else
    --knee pitch should be positive
    qLegs[4]= math.max(0,qLegs[4])
    qLegs[10]= math.max(0,qLegs[10])  
  end

  local legBias = vector.new(mcm.get_leg_bias())
  qLegs = qLegs + delta_legs + legBias
  qLegs[5]=qLegs[5]
  qLegs[11]=qLegs[11]
  qLegs[6]=qLegs[6]
  qLegs[12]=qLegs[12]

  if Config.walk.anklePitchLimit then
    qLegs[5]=math.max(qLegs[5],Config.walk.anklePitchLimit[1])
    qLegs[11]=math.max(qLegs[11],Config.walk.anklePitchLimit[1])
    qLegs[5]=math.min(qLegs[5],Config.walk.anklePitchLimit[2])
    qLegs[11]=math.min(qLegs[11],Config.walk.anklePitchLimit[2])
  end
--
  --hip pitch lag fix
  if Config.walk.hipPitch0 then
    if qLegs[3]<Config.walk.hipPitch0 then
      qLegs[3] = Config.walk.hipPitch0 + (qLegs[3]-Config.walk.hipPitch0)*Config.walk.hipPitchCompensationMag
    end
    if qLegs[9]<Config.walk.hipPitch0 then
      qLegs[9] = Config.walk.hipPitch0 + (qLegs[9]-Config.walk.hipPitch0)*Config.walk.hipPitchCompensationMag
    end
  end


  local q1,q2,q3 = moveleg.joint_trajectory_kick(phSingle)

  q2 = q2 + 10*DEG_TO_RAD

  local blend_factor = math.min(1, phSingle/0.2)
  blend_factor = math.min(blend_factor,(1-phSingle)/0.2)

  if supportLeg==0 then --left support, right kick
    qLegs[9],qLegs[10],qLegs[11] = 
    qLegs[9]*(1-blend_factor) + q1*blend_factor,
    qLegs[10]*(1-blend_factor) + q2*blend_factor,
    qLegs[11]*(1-blend_factor) + q3*blend_factor
  else
    qLegs[3],qLegs[4],qLegs[5] = 
    qLegs[3]*(1-blend_factor) + q1*blend_factor,
    qLegs[4]*(1-blend_factor) + q2*blend_factor,
    qLegs[5]*(1-blend_factor) + q3*blend_factor
  end







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
  if IS_WEBOTS then
    local llt = Body.get_lleg_current()
    local rlt = Body.get_rleg_current() 

    mcm.set_status_lleg_torque(llt)
    mcm.set_status_rleg_torque(rlt)
  end
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
