---------------------------------------------------------
-- Central pattern generator walk based on robotis walk code
--------------------------------------------------------


local walk = {}
walk._NAME = ...

local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local moveleg = require'moveleg'
require'mcm'

-- Simple IPC for remote state triggers
local simple_ipc = require'simple_ipc'
local evts = simple_ipc.new_subscriber('Walk',true)

-- Keep track of important times
local t_entry, t_update, t_last_step

-- Stance parameters
local bodyTilt = Config.walk.bodyTilt or 0
local torsoX   = Config.walk.torsoX
local footY    = Config.walk.footY

--Gait parameters
local stepHeight  = Config.walk.stepHeight
local tStep = Config.walk.tStep
---------------------------------------------------------
-- Walk state variables
-- These are continuously updated on each update
----------------------------------------------------------
-- Save the velocity between update cycles
local velCurrent = vector.new{0, 0, 0}

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local ankleShift = vector.new{0, 0}
local kneeShift  = 0
local hipShift   = vector.new{0, 0}

-- Still have an initial step for now
local initial_step, iStep

local function wsin(t,period,period_shift,mag,mag_shift)
  return mag*math.sin(t * (2*math.pi/period) - period_shift) + mag_shift
end

local function get_movement(t,period,period_shift,t1,mag,mag_shift)
  return wsin(t,period,period_shift+ 2*math.pi/period *t1,  mag, mag_shift)
end

---------------Robotis walk parameters
local x_offset = 0
local y_offset = 0.075
local z_offset = 0.060
local hip_pitch_offset = 11*math.pi/180
local period_time = 0.900
local dsp_ratio = 0.25
local step_fb_ratio = 0.32
local foot_height = 0.050
local swing_lr = 0.050
local swing_ud = 0.005
local balance_knee_gain = 0.3
local balance_ankleX_gain = 0.9
local balance_hipY_gain = 0.5
local balance_ankleY_gain = 1.0
local balance_ankle_IX_gain = 0.5
local balance_ankle_IY_gain = 1.0
--------------------------------------

tStep = period_time


--From IK parameters
PELVIS_Y = 0.072 
LEG_MAX_DIST = 0.30149627 + 0.30149627 + 0.118
WAIST_Z = 0.270

local footz0 = z_offset + 2*swing_ud -(foot_height)/4
local bodyHeight_converted = LEG_MAX_DIST+WAIST_Z*math.cos(hip_pitch_offset) - footz0;
local supportX_converted = WAIST_Z*math.sin(hip_pitch_offset)
local footY_converted = PELVIS_Y + y_offset/2

print("Converted parameters:")
print("bodyHeight:",bodyHeight_converted) 
print("supportX:",supportX_converted)
print("footY:",footY_converted)

local function init_params()
  ssp_ratio = 1-dsp_ratio
  t_SSP = period_time * ssp_ratio
  t_SSP_start_L = (1-ssp_ratio)/4 * period_time
  t_SSP_end_L = (1+ssp_ratio)/4 * period_time
  t_SSP_start_R = (3-ssp_ratio)/4 * period_time
  t_SSP_end_R = (3+ssp_ratio)/4 * period_time
 
  pt_X = ssp_ratio * period_time
  pt_Y = ssp_ratio * period_time
  pt_Z = 0.5* ssp_ratio * period_time
  pt_A = ssp_ratio * period_time   

  pt_swap_X = 0.5*period_time
  pt_swap_Y = period_time
  pt_swap_Z = 0.5 * period_time

  dPt_swap_X = math.pi
  dPt_swap_Y = 0
  dPt_swap_Z = math.pi*1.5 

  dPt_X = math.pi*0.5
  dPt_Y = math.pi*0.5
  dPt_Z = math.pi*0.5
  dPt_A = math.pi*0.5

  offset_pelvis = 0
  swing_pelvis = offset_pelvis*0.35
end

local function update_params()
  mag_X = velCurrent[1]
  dMag_X = 0
  mag_Y = velCurrent[2]/2  
  if mag_Y>0 then dMag_Y = mag_Y else dMag_Y = -mag_Y end
  mag_Z = foot_height/2
  dMag_Z = mag_Z/2

  mag_swap_X = velCurrent[1] * step_fb_ratio
  dMag_swap_X = 0
  mag_swap_Y = swing_lr + dMag_Y * 0.04
  dMag_swap_Y = 0
  mag_swap_Z = swing_ud
  dMag_swap_Z = mag_swap_Z

  mag_A=velCurrent[3]/2
  if (mag_A>0) then dMag_A = mag_A else dMag_A = -mag_A end

end

local function calculate_torso_movement(t)
  x_swap = wsin(t, pt_swap_X, dPt_swap_X, mag_swap_X, dMag_swap_X);
  y_swap = wsin(t, pt_swap_Y, dPt_swap_Y, mag_swap_Y, dMag_swap_Y);
  z_swap = wsin(t, pt_swap_Z, dPt_swap_Z, mag_swap_Z, dMag_swap_Z);
  
  if (t<t_SSP_start_L) then --Double support phase
    x_move_l = get_movement(t_SSP_start_L, pt_X, dPt_X, t_SSP_start_L,mag_X, dMag_X)
    y_move_l = get_movement(t_SSP_start_L, pt_Y, dPt_Y, t_SSP_start_L,mag_Y, dMag_Y)
    z_move_l = get_movement(t_SSP_start_L, pt_Z, dPt_Z, t_SSP_start_L,mag_Z, dMag_Z)
    c_move_l = get_movement(t_SSP_start_L, pt_A, dPt_A, t_SSP_start_L,mag_A, dMag_A)

    x_move_r = get_movement(t_SSP_start_L, pt_X, dPt_X, t_SSP_start_L,-mag_X, -dMag_X)
    y_move_r = get_movement(t_SSP_start_L, pt_Y, dPt_Y, t_SSP_start_L,-mag_Y, -dMag_Y)
    z_move_r = get_movement(t_SSP_start_R, pt_Z, dPt_Z, t_SSP_start_R,mag_Z, dMag_Z)
    c_move_r = get_movement(t_SSP_start_L, pt_A, dPt_A, t_SSP_start_L,-mag_A, -dMag_A)
    pelvis_offset_l = 0
    pelvis_offset_r = 0
  elseif t<t_SSP_end_L then --Single support phase, Right support

  update_params()
    x_move_l = get_movement(t, pt_X, dPt_X ,t_SSP_start_L, mag_X, dMag_X)
    y_move_l = get_movement(t, pt_Y, dPt_Y ,t_SSP_start_L, mag_Y, dMag_Y)
    z_move_l = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_L, mag_Z, dMag_Z)
    c_move_l = get_movement(t, pt_A, dPt_A ,t_SSP_start_L, mag_A, dMag_A)

    x_move_r = get_movement(t, pt_X, dPt_X ,t_SSP_start_L, -mag_X, -dMag_X)
    y_move_r = get_movement(t, pt_Y, dPt_Y ,t_SSP_start_L, -mag_Y, -dMag_Y)
    z_move_r = get_movement(t_SSP_start_R, pt_Z, dPt_Z ,t_SSP_start_R, mag_Z, dMag_Z)
    c_move_r = get_movement(t, pt_A, dPt_A ,t_SSP_start_L, -mag_A, -dMag_A)

    pelvis_offset_l = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_L, swing_pelvis/2, swing_pelvis/2)
    pelvis_offset_r = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_L, -offset_pelvis/2, -offset_pelvis/2)
  elseif t<t_SSP_start_R then --Double support phase
    x_move_l = get_movement(t_SSP_end_L, pt_X, dPt_X, t_SSP_start_L,mag_X, dMag_X)
    y_move_l = get_movement(t_SSP_end_L, pt_Y, dPt_Y, t_SSP_start_L,mag_Y, dMag_Y)
    z_move_l = get_movement(t_SSP_end_L, pt_Z, dPt_Z, t_SSP_start_L,mag_Z, dMag_Z)
    c_move_l = get_movement(t_SSP_end_L, pt_A, dPt_A, t_SSP_start_L,mag_A, dMag_A)

    x_move_r = get_movement(t_SSP_end_L, pt_X, dPt_X, t_SSP_start_L,-mag_X, -dMag_X)
    y_move_r = get_movement(t_SSP_end_L, pt_Y, dPt_Y, t_SSP_start_L,-mag_Y, -dMag_Y)
    z_move_r = get_movement(t_SSP_start_R, pt_Z, dPt_Z, t_SSP_start_R,mag_Z, dMag_Z)
    c_move_r = get_movement(t_SSP_end_L, pt_A, dPt_A, t_SSP_start_L,-mag_A, -dMag_A)
    pelvis_offset_l = 0
    pelvis_offset_r = 0
  elseif t<t_SSP_end_R then  --Single support phase, Left support
    x_move_l = get_movement(t, pt_X, dPt_X ,t_SSP_start_R+math.pi, mag_X, dMag_X)
    y_move_l = get_movement(t, pt_Y, dPt_Y ,t_SSP_start_R+math.pi, mag_Y, dMag_Y)
    z_move_l = get_movement(t_SSP_end_L, pt_Z, dPt_Z ,t_SSP_start_L, mag_Z, dMag_Z)
    c_move_l = get_movement(t, pt_A, dPt_A ,t_SSP_start_R+math.pi, mag_A, dMag_A)

    x_move_r = get_movement(t, pt_X, dPt_X ,t_SSP_start_R+math.pi, -mag_X, -dMag_X)
    y_move_r = get_movement(t, pt_Y, dPt_Y ,t_SSP_start_R+math.pi, -mag_Y, -dMag_Y)
    z_move_r = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_R, mag_Z, dMag_Z)
    c_move_r = get_movement(t, pt_A, dPt_A ,t_SSP_start_R+math.pi, -mag_A, -dMag_A)

    pelvis_offset_l = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_R, offset_pelvis/2, offset_pelvis/2)
    pelvis_offset_r = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_R, -swing_pelvis/2, -swing_pelvis/2)
  else --Double support phase
    x_move_l = get_movement(t_SSP_end_R, pt_X, dPt_X, t_SSP_start_R+math.pi,mag_X, dMag_X)
    y_move_l = get_movement(t_SSP_end_R, pt_Y, dPt_Y, t_SSP_start_R+math.pi,mag_Y, dMag_Y)
    z_move_l = get_movement(t_SSP_end_L, pt_Z, dPt_Z, t_SSP_start_L,mag_Z, dMag_Z)
    c_move_l = get_movement(t_SSP_end_R, pt_A, dPt_A, t_SSP_start_R+math.pi,mag_A, dMag_A)

    x_move_r = get_movement(t_SSP_end_R, pt_X, dPt_X, t_SSP_start_R+math.pi,-mag_X, -dMag_X)
    y_move_r = get_movement(t_SSP_end_R, pt_Y, dPt_Y, t_SSP_start_R+math.pi,-mag_Y, -dMag_Y)
    z_move_r = get_movement(t_SSP_end_R, pt_Z, dPt_Z, t_SSP_start_R,mag_Z, dMag_Z)
    c_move_r = get_movement(t_SSP_end_R, pt_A, dPt_A, t_SSP_start_R+math.pi,-mag_A, -dMag_A)
    pelvis_offset_l = 0
    pelvis_offset_r = 0
  end

  trLLeg = {
    x_swap + x_move_l + x_offset,
    y_swap + y_move_l + y_offset /2 + PELVIS_Y,
    z_swap + z_move_l + z_offset,
    0,0,c_move_l
  }
  trRLeg = {
    x_swap + x_move_r + x_offset,
    y_swap + y_move_r - y_offset /2 - PELVIS_Y,
    z_swap + z_move_r + z_offset,
    0,0,c_move_r
  }

  return trLLeg, trRLeg
end


---------------------------
-- State machine methods --
---------------------------
function walk.entry()
  print(walk._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Reset our velocity
  velCurrent = vector.new{0,0,0}
  mcm.set_walk_vel(velCurrent)

  --Read stored feet and torso poses 
  local uTorso0 = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  uTorso_now, uTorso_next = uTorso0, uTorso0
  uLeft_now,  uLeft_next  = uLeft,  uLeft
  uRight_now, uRight_next = uRight, uRight

  t_last_step = Body.get_time()-tStep --for starting the next step right now

  iStep = 1   -- Initialize the step index  
  initial_step = 0 -- We don't need this for static walk
  
  mcm.set_walk_bipedal(1)
  mcm.set_walk_stoprequest(0) --cancel stop request flag

  init_params()
  update_params()
end

function walk.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

  --SJ: walk events are simply handled via SHM 
  local stoprequest = mcm.get_walk_stoprequest()

  local ph = (t-t_last_step)/tStep
  if ph>1 then
    ph = ph % 1
    if stoprequest>0 then return"done" end --Should we stop now?
    velCurrent = moveleg.update_velocity(velCurrent)     -- Update the velocity via a filter
  end
  
  local gyro_rpy = Body.get_sensor_gyro()

  delta_legs, ankleShift, kneeShift, hipShift = moveleg.get_leg_compensation(
      3,0,
      gyro_rpy, 
      ankleShift, kneeShift, hipShift, 
      0)
  
  pLLeg, pRLeg = calculate_torso_movement(ph*tStep)

  local pTorso = {supportX_converted,0,bodyHeight_converted,   0,hip_pitch_offset,0}
  pLLeg[3] = pLLeg[3] - footz0;
  pRLeg[3] = pRLeg[3] - footz0;
  supportLeg = 2

  moveleg.set_leg_positions(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)  


--[[
  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  --------------------------------------------------------------------
  phFootSingle1 = 0.3
  phFootSingle2 = 0.7
  local xFoot, zFoot, phSingle = moveleg.get_foot_square(
      ph,phFootSingle1,phFootSingle2)  
  --Don't lift foot at initial step
  if initial_step>0 then zFoot = 0  end

  -- Begin to solve for our leg positions  
  local uLeft, uRight, zLeft, zRight
  if supportLeg == 0 then
    -- Left support
    uLeft = uLeft_now 
    uRight = util.se2_interpolate(xFoot, uRight_now, uRight_next)
    zLeft = 0
    zRight =  stepHeight*zFoot
  else
    -- Right support
    uRight = uRight_now 
    uLeft = util.se2_interpolate(xFoot, uLeft_now, uLeft_next)
    zLeft =  stepHeight*zFoot    
    zRight = 0
  end

  -- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local delta_legs
  delta_legs, ankleShift, kneeShift, hipShift = moveleg.get_leg_compensation(
      supportLeg,phSingle,gyro_rpy, ankleShift, kneeShift, hipShift, initial_step)

  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], Config.walk.bodyHeight,
        0,bodyTilt,uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
    
  moveleg.set_leg_positions(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)  
  
  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------
  --]]
end -- walk.update

function walk.exit()
  mcm.set_status_uLeft(uLeft_next)
  mcm.set_status_uRight(uRight_next)
  mcm.set_status_uTorso(uTorso_next)
  print(walk._NAME..' Exit')  
end

return walk
