---------------------------------------------------------
-- Central pattern generator walk based on robotis walk code
--------------------------------------------------------

local walk = {}
walk._NAME = ...

local Body   = require'Body'
local vector = require'vector'
local util   = require'util'
local moveleg = require'moveleg'
local libStep = require'libStep'
local step_planner
require'mcm'

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
local angleShift = vector.new{0,0,0,0}

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

local function update_params(velCurrent)
  print(velCurrent)
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
--    print("DS")
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
--  print("RS_SS")
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
--    print("DS")    
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
--  print("LS_SS")
    x_move_l = get_movement(t,           pt_X, dPt_X ,t_SSP_start_R+math.pi, mag_X, dMag_X)
    y_move_l = get_movement(t,           pt_Y, dPt_Y ,t_SSP_start_R+math.pi, mag_Y, dMag_Y)
    z_move_l = get_movement(t_SSP_end_L, pt_Z, dPt_Z ,t_SSP_start_L,         mag_Z, dMag_Z)
    c_move_l = get_movement(t,           pt_A, dPt_A ,t_SSP_start_R+math.pi, mag_A, dMag_A)

    x_move_r = get_movement(t,           pt_X, dPt_X ,t_SSP_start_R+math.pi, -mag_X, -dMag_X)
    y_move_r = get_movement(t,           pt_Y, dPt_Y ,t_SSP_start_R+math.pi, -mag_Y, -dMag_Y)
    z_move_r = get_movement(t,           pt_Z, dPt_Z ,t_SSP_start_R, mag_Z, dMag_Z)
    c_move_r = get_movement(t,           pt_A, dPt_A ,t_SSP_start_R+math.pi, -mag_A, -dMag_A)

    pelvis_offset_l = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_R, offset_pelvis/2, offset_pelvis/2)
    pelvis_offset_r = get_movement(t, pt_Z, dPt_Z ,t_SSP_start_R, -swing_pelvis/2, -swing_pelvis/2)
  else --Double support phase
--    print("DS")
    x_move_l = get_movement(t_SSP_end_R, pt_X, dPt_X, t_SSP_start_R+math.pi,mag_X, dMag_X)
    y_move_l = get_movement(t_SSP_end_R, pt_Y, dPt_Y, t_SSP_start_R+math.pi,mag_Y, dMag_Y)
    z_move_l = get_movement(t_SSP_end_L, pt_Z, dPt_Z, t_SSP_start_L,        mag_Z, dMag_Z)
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


  step_planner = libStep.new_planner()
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
  update_params(velCurrent)
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
    step_planner:update_velocity(mcm.get_walk_vel())
    update_params(step_planner.velCurrent)    
    t_last_step = t
  end

  pLLeg, pRLeg = calculate_torso_movement(ph*tStep)
  
  local gyro_rpy = Body.get_sensor_gyro()
  delta_legs, angleShift = moveleg.get_leg_compensation(3,0,gyro_rpy, angleShift)

  local pTorso = {supportX_converted,0,bodyHeight_converted,   0,hip_pitch_offset,0}
  pLLeg[3] = pLLeg[3] - footz0;
  pRLeg[3] = pRLeg[3] - footz0;
  supportLeg = 2

  moveleg.set_leg_transforms(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)  
end -- walk.update

function walk.exit()
  mcm.set_status_uLeft(uLeft_next)
  mcm.set_status_uRight(uRight_next)
  mcm.set_status_uTorso(uTorso_next)
  print(walk._NAME..' Exit')  
end

return walk
