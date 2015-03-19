--Stance state is basically a Walk controller
--Without any torso or feet update
--We share the leg joint generation / balancing code 
--with walk controllers

local state = {}
state._NAME = ...

local mcm     = require'mcm'
local Body    = require'Body'
local vector  = require'vector'
local unix    = require'unix'
local util    = require'util'
local moveleg = require'moveleg'
local movearm = require'movearm'
local T       = require'Transform'
local atan2   = math.atan2
local sqrt    = math.sqrt
local cos     = math.cos
local sin     = math.sin


--local webot = require'webot'

require'mcm'
require'hcm'

local simple_ipc = require'simple_ipc'
local head_ch   = simple_ipc.new_publisher('HeadFSM!')

local qLArm0, qRArm0,qLLeg0,qRLeg0,qWaist0
local qLArm1, qRArm1,qLLeg1,qRLeg1,qWaist1

local stage = 0
local DEG_TO_RAD = math.pi/180
---------------------------
-- State machine methods --
---------------------------
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_start = t_entry
  
  qLArm0, qLArm1 = vector.new(Body.get_larm_command_position())  
  qRArm0, qRArm1 = vector.new(Body.get_rarm_command_position())  
  qLLeg0, qLLeg1 = vector.new(Body.get_lleg_command_position())  
  qRLeg0, qRLeg1 = vector.new(Body.get_rleg_command_position())  
  qWaist0, qWaist1 = vector.new(Body.get_waist_command_position())  

--print(unpack(vector.new(qLArm1)/DEG_TO_RAD))
  setting = 1
  stage = 1  
  hcm.set_state_proceed(1)
  head_ch:send'teleop'
  body_ch:send'init'

end



function state.update()
  -- Get the time of update
  --local torque=webot.wb_motor_get_torque_feedback()
  local t = Body.get_time()
  print("time getting from Body:",t)
  local t_diff = t - t_start
  print("t_start:",t_start)



  local rpy = Body.get_rpy()
  print("R",rpy[1]/DEG_TO_RAD)
  print("P",rpy[2]/DEG_TO_RAD)
  print("Y",rpy[3]/DEG_TO_RAD)

  local ori_ctrl_larm = 0
  local ori_ctrl_rarm = 0
  local ori_ctrl_lleg = 0
  local ori_ctrl_rleg = 0

  
  -- Check wheter the robot was fallen down or not
  if math.abs(rpy[1])<45*math.pi/180 and math.abs(rpy[2])<45*math.pi/180 then      
    return "done"
  end
  
  --[[Get Position vectors of all joints]]----------------------------------------------------------------------------

  qLArm_current   = Body.get_larm_position();
  qRArm_current   = Body.get_rarm_position();
  qLLeg_current   = Body.get_lleg_position();
  qRLeg_current   = Body.get_rleg_position();
  qWaist_current  = Body.get_waist_position();
  qHead_current   = Body.get_head_position();

  currentLArm = Body.get_forward_larm(qLArm_current)
  currentRArm = Body.get_forward_rarm(qRArm_current)
  currentLLeg = Body.get_forward_lleg(qLLeg_current)
  currentRLeg = Body.get_forward_rleg(qRLeg_current)

  --[[ Rotation Matrices ]]--------------------------------------------------------------------------------------------

  rpyR_b2g ={} -- RPY rotation matrix of body w.r.t the global frame
    rpyR_b2g[1] = {}
      rpyR_b2g[1][1] = cos(rpy[1])*cos(rpy[2])
      rpyR_b2g[1][2] = cos(rpy[1])*sin(rpy[2])*sin(rpy[3]) - sin(rpy[1])*cos(rpy[3])
      rpyR_b2g[1][3] = cos(rpy[1])*sin(rpy[2])*cos(rpy[3]) + sin(rpy[1])*sin(rpy[3])
    rpyR_b2g[2] = {}
      rpyR_b2g[2][1] = sin(rpy[1])*cos(rpy[2])
      rpyR_b2g[2][2] = sin(rpy[1])*sin(rpy[2])*sin(rpy[3]) + cos(rpy[1])*cos(rpy[3])
      rpyR_b2g[2][3] = sin(rpy[1])*sin(rpy[2])*cos(rpy[3]) - cos(rpy[1])*sin(rpy[3]) 
    rpyR_b2g[3] = {}
      rpyR_b2g[3][1] = -sin(rpy[2])
      rpyR_b2g[3][2] = cos(rpy[2])*sin(rpy[3])
      rpyR_b2g[3][3] = cos(rpy[2])*cos(rpy[3])

  rpy_target = {}
  rpy_target[1] = 0--roll
  rpy_target[2] = 0 --pitch
  rpy_target[3] = rpy[3] --yaw
  rpyR_tr2g = {} -- RPY rotation matrix of the end-effector with targeted orientations w.r.t the global frame(super script)
    rpyR_tr2g[1] = {}
      rpyR_tr2g[1][1] = cos(rpy_target[1])*cos(rpy_target[2])
      rpyR_tr2g[1][2] = cos(rpy_target[1])*sin(rpy_target[2])*sin(rpy_target[3]) - sin(rpy_target[1])*cos(rpy_target[3])
      rpyR_tr2g[1][3] = cos(rpy_target[1])*sin(rpy_target[2])*cos(rpy_target[3]) + sin(rpy_target[1])*sin(rpy_target[3])
    rpyR_tr2g[2] = {}
      rpyR_tr2g[2][1] = sin(rpy_target[1])*cos(rpy_target[2])
      rpyR_tr2g[2][2] = sin(rpy_target[1])*sin(rpy_target[2])*sin(rpy_target[3]) + cos(rpy_target[1])*cos(rpy_target[3])
      rpyR_tr2g[2][3] = sin(rpy_target[1])*sin(rpy_target[2])*cos(rpy_target[3]) - cos(rpy_target[1])*sin(rpy_target[3]) 
    rpyR_tr2g[3] = {}
      rpyR_tr2g[3][1] = -sin(rpy_target[2])
      rpyR_tr2g[3][2] = cos(rpy_target[2])*sin(rpy_target[3])
      rpyR_tr2g[3][3] = cos(rpy_target[2])*cos(rpy_target[3])

  rpyR_tr2b = util.mul(util.transpose(rpyR_b2g),rpyR_tr2g)

--------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------


--[[ Find the lowest joint ]]---------------------------------------------------------------------------------------------

  numJoint = {};
  numJoint[1] = 7
  numJoint[2] = 7
  numJoint[3] = 6
  numJoint[4] = 6

  j_heights = {}
  local jointCount = 0
  
  for i=1,numJoint[1] do
      or_pos = Body.get_forward_larm_origins(qLArm_current,i)
      j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
  end
  jointCount = jointCount + numJoint[1]

  for i=1,numJoint[2] do
      or_pos = Body.get_forward_rarm_origins(qRArm_current,i)
      j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
  end
  jointCount = jointCount + numJoint[2]

  for i=1,numJoint[3] do
      or_pos = Body.get_forward_lleg_origins(qLLeg_current,i)
      j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
  end
  jointCount = jointCount + numJoint[3]

  for i=1,numJoint[4] do
      or_pos = Body.get_forward_rleg_origins(qRLeg_current,i)
      j_heights[jointCount+i] = rpyR_b2g[3][1]*or_pos[1] + rpyR_b2g[3][2]*or_pos[2] + rpyR_b2g[3][3]*or_pos[3]
  end
  jointCount = jointCount + numJoint[4]

  baseframe_h = util.min(j_heights)

----------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------
  --[[
local legBiasR, legBiasL = vector.zeros(7), vector.zeros(7)
function moveleg.update_leg_bias()
  -- TODO: Fix the bias issue
  local legBias = 0*mcm.get_leg_bias()
  legBiasL = vector.slice(legBias, 1, 6)
  legBiasR = vector.slice(legBias, 7, 12)
end
]]
--[[ stages ]]--------------------------------------------------------------------------------------------------

  
  stagemax = 5
  stop = 0

  if stage == 1 and setting == 1 then
    --*** Stage parameters ***--
    init = 1
    duration = 3
    --1) Support Parts for the current STAGE
    supportIdx = {}
    j=0
    for i=1,jointCount do
      if math.abs(j_heights[i]-baseframe_h) < 0.05 then
        supportIdx[j] = i
        j = j+1
      end
    end
    -- If : support parts include left or/and right hand -> move up first

    --2) Compute the desired positions 
    l_shoulder = Body.get_forward_larm_origins(qLArm_current,2)
    r_shoulder = Body.get_forward_rarm_origins(qRArm_current,2)

    l_pos_sh = {} -- 3 by 1
      l_pos_sh[1] = l_shoulder[1]
      l_pos_sh[2] = l_shoulder[2]
      l_pos_sh[3] = l_shoulder[3]

    r_pos_sh = {} -- 3 by 1
      r_pos_sh[1] = r_shoulder[1]
      r_pos_sh[2] = r_shoulder[2]
      r_pos_sh[3] = r_shoulder[3]

    l_pos_eff = {}
      l_pos_eff[1] = currentLArm[1]
      l_pos_eff[2] = currentLArm[2]
      l_pos_eff[3] = currentLArm[3]

    r_pos_eff = {}
      r_pos_eff[1] = currentRArm[1]
      r_pos_eff[2] = currentRArm[2]
      r_pos_eff[3] = currentRArm[3]

    l_shoulder2g  = util.mulvec(rpyR_b2g,l_pos_sh) -- 3 by 1
    r_shoulder2g  = util.mulvec(rpyR_b2g,r_pos_sh) -- 3 by 1

    l_current2g   = util.mulvec(rpyR_b2g,l_pos_eff)
    r_current2g   = util.mulvec(rpyR_b2g,r_pos_eff)

    l_trPos2g = {}
      l_trPos2g[1] = l_shoulder2g[1]+0.25
      l_trPos2g[2] = l_shoulder2g[2]+util.sign(l_shoulder2g[2])*0.03
      l_trPos2g[3] = baseframe_h-0.15

    r_trPos2g = {}
      r_trPos2g[1] = r_shoulder2g[1]+0.25
      r_trPos2g[2] = r_shoulder2g[2]+util.sign(r_shoulder2g[2])*0.03
      r_trPos2g[3] = baseframe_h-0.15

    l_trPos_init2g = {}
      l_trPos_init2g[1] = (l_trPos2g[1] + l_current2g[1])/2
      l_trPos_init2g[2] = (l_trPos2g[2] + l_current2g[2])/2
      l_trPos_init2g[3] = l_current2g[3] + 0.25
    
    r_trPos_init2g = {}
      r_trPos_init2g[1] = (r_trPos2g[1] + r_current2g[1])/2
      r_trPos_init2g[2] = (r_trPos2g[2] + r_current2g[2])/2
      r_trPos_init2g[3] = r_current2g[3] + 0.25


    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)

    l_trPos_init = util.mulvec(util.transpose(rpyR_b2g),l_trPos_init2g)
    r_trPos_init = util.mulvec(util.transpose(rpyR_b2g),r_trPos_init2g)

    trL_init = {}
    trL_init[1] = l_trPos_init[1]
    trL_init[2] = l_trPos_init[2]
    trL_init[3] = l_trPos_init[3]
    trL_init[4] = (atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) + currentLArm[4])/2 --yaw
    trL_init[5] = (atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + currentLArm[5])/2 --pitch
    trL_init[6] = (atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) + currentLArm[6])/2 --roll : but about x-axis

    trR_init = {}
    trR_init[1] = r_trPos_init[1]
    trR_init[2] = r_trPos_init[2]
    trR_init[3] = r_trPos_init[3]
    trR_init[4] = (atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])+ currentRArm[4])/2--yaw
    trR_init[5] = (atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )+ currentRArm[5])/2--pitch
    trR_init[6] = (-atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) + currentRArm[6])/2--roll

    trL = {}
    trL[1] = l_trPos[1]
    trL[2] = l_trPos[2]
    trL[3] = l_trPos[3]
    trL[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trL[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trL[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll
    
    if trL[6] > math.pi/2 then
      trL_init[6] = currentLArm[6]
      trR_init[6] = currentRArm[6]
    end

    trR = {}
    trR[1] = r_trPos[1]
    trR[2] = r_trPos[2]
    trR[3] = r_trPos[3]
    trR[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trR[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trR[6] = -atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD
    
    qLLeg1 = vector.new(Body.get_lleg_position())
    qRLeg1 = vector.new(Body.get_rleg_position())
    qWaist1 = vector.new(Body.get_waist_position())

    -- Init
    qLArm_init = vector.new(Body.get_inverse_larm(qLArm_current, trL_init))
    --qLArm_init[7] = qLArm_init[7] + 90*DEG_TO_RAD
    qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
    --qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD

    qLLeg_init = qLLeg1
    qRLeg_init = qRLeg1
    qWaist_init = qWaist1

    setting = 0

  elseif stage == 2 and setting == 1 then
    --*** Stage parameters ***--

    init = 0
    duration = 2

    l_trPos2g[3] = baseframe_h-0.25
    r_trPos2g[3] = baseframe_h-0.25

    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    l_trPos[3] = l_trPos[3] + 0.1 -- farther from base fame (vertical body)
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
    r_trPos[3] = r_trPos[3] + 0.1

    pitch_offset = 12*DEG_TO_RAD
    trL = {}
    trL[1] = l_trPos[1]
    trL[2] = l_trPos[2]
    trL[3] = l_trPos[3]
    trL[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trL[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset --pitch
    trL[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    trR = {}
    trR[1] = r_trPos[1]
    trR[2] = r_trPos[2]
    trR[3] = r_trPos[3]
    trR[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trR[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset--pitch
    trR[6] = -atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD
    
    qLLeg1 = vector.new(Body.get_lleg_position())
    qRLeg1 = vector.new(Body.get_rleg_position())
    qWaist1 = vector.new(Body.get_waist_position())

    setting = 0

  elseif stage == 3 and setting == 1 then
    print("STAGE START!:",stage)
    init = 1
    duration = 5
    
    l_trPos2g[3] = l_trPos2g[3] - 0.1
    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    l_trPos[3] = l_trPos[3] - 0.1 -- pull arm 
    l_trPos[2] = l_trPos[2] - util.sign(l_trPos[2])*0.05 -- move closer along with horizontal axis

    pitch_offset = 20*DEG_TO_RAD
    trL = {}
    trL[1] = l_trPos[1]
    trL[2] = l_trPos[2]
    trL[3] = l_trPos[3]
    trL[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trL[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset --pitch
    trL[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll


    l_trPos_init2g = l_trPos2g
    l_trPos_init2g[3] = l_trPos2g[3]/2
    l_trPos_init = util.mulvec(util.transpose(rpyR_b2g),l_trPos_init2g)

    trL_init = {}
    trL_init[1] = l_trPos_init[1]
    trL_init[2] = l_trPos_init[2]
    trL_init[3] = l_trPos_init[3]
    trL_init[4] = (atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) + currentLArm[4])/2 --yaw
    trL_init[5] = (atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + currentLArm[5])/2 --pitch
    trL_init[6] = (atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) + currentLArm[6])/2 --roll : but about x-axis

    --------Right Leg

    stage_h = baseframe_h+0.016    

    r_hip = Body.get_forward_rleg_origins(qRLeg_current,2)
    r_foot = Body.get_forward_rleg_origins(qRLeg_current,6)

    r_pos_hip = {} -- 3 by 1
      r_pos_hip[1] = r_hip[1]
      r_pos_hip[2] = r_hip[2]
      r_pos_hip[3] = r_hip[3]

    rleg_pos_eff = {}
      rleg_pos_eff[1] = r_foot[1]
      rleg_pos_eff[2] = r_foot[2]
      rleg_pos_eff[3] = r_foot[3]

    r_hip2g  = util.mulvec(rpyR_b2g,r_pos_hip) -- 3 by 1
    rleg_current2g   = util.mulvec(rpyR_b2g,rleg_pos_eff)

    rleg_trPos2g = {}
      rleg_trPos2g[1] = r_hip2g[1]
      rleg_trPos2g[2] = r_hip2g[2]
      rleg_trPos2g[3] = stage_h

    rleg_trPos_init2g = {}
      rleg_trPos_init2g[1] = rleg_current2g[1]
      rleg_trPos_init2g[2] = rleg_current2g[2]
      rleg_trPos_init2g[3] = rleg_current2g[3]+0.005

    rleg_trPos = util.mulvec(util.transpose(rpyR_b2g), rleg_trPos2g)
    rleg_trPos[3] = rleg_trPos[3] - 0.1 -- negative farther than hip joint (vertical body)  util.sign(rleg_trPos[3])*
    rleg_trPos[2] = rleg_trPos[2] -- util.sign(rleg_trPos[2])*0.05
    
    rleg_trPos_init = util.mulvec(util.transpose(rpyR_b2g),rleg_trPos_init2g)
    rleg_trPos_init[3] = rleg_trPos_init[3] + 0.05 -- closer than current right foot joint (vertical body) util.sign(rleg_trPos_init[3])*
    rleg_trPos_init[2] = rleg_trPos_init[2] - util.sign(rleg_trPos_init[2])*0.05

    trRLeg_init = {}
    trRLeg_init[1] = rleg_trPos_init[1]
    trRLeg_init[2] = rleg_trPos_init[2]
    trRLeg_init[3] = rleg_trPos_init[3]
    trRLeg_init[4] = (currentRLeg[4] )--+ atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]))/2 --yaw
    trRLeg_init[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) --pitch
    trRLeg_init[6] = (currentRLeg[6]) --+ atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]))/2 --roll : but about x-axis

    trRLeg = {}
    trRLeg[1] = rleg_trPos[1]
    trRLeg[2] = rleg_trPos[2]
    trRLeg[3] = rleg_trPos[3]
    trRLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trRLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trRLeg[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll
    --------

    fold_ang = 20

    qLLeg1 = vector.new(Body.get_lleg_position())
      qLLeg1[3] = qLLeg1[3]+util.sign(qLLeg1[3])*fold_ang*DEG_TO_RAD
      qLLeg1[4] = qLLeg1[4]+util.sign(qLLeg1[4])*fold_ang*DEG_TO_RAD
    --qRLeg1 = vector.new(Body.get_rleg_position())
    --  qRLeg1[3] = qRLeg1[3]+util.sign(qRLeg1[3])*fold_ang*DEG_TO_RAD
    --  qRLeg1[4] = qRLeg1[4]+util.sign(qRLeg1[4])*fold_ang*DEG_TO_RAD
    qRLeg1 = vector.new(Body.get_inverse_rleg(trRLeg))
    qWaist1 = vector.new(Body.get_waist_position())
      qWaist1[2] = qWaist1[2] - fold_ang*0.6*DEG_TO_RAD
      --qWaist1[1] = -20*DEG_TO_RAD
    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    --qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL, qLArm_current[3], qWaist1, qWaist1[1]))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_rarm_position())

    qLArm_init = vector.new(Body.get_inverse_larm(qLArm_current, trL_init))
      qLArm_init[7] = qLArm_init[7] + 90*DEG_TO_RAD
    --qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
      --qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD
    qRArm_init = qRArm1
    
    qLLeg_init = (qLLeg0 + qLLeg1)/2
    --qRLeg_init = (qRLeg0 + qRLeg1)/2
    qRLeg_init = vector.new(Body.get_inverse_rleg(trRLeg_init))
    qWaist_init = (qWaist0 + qWaist1)/2


    setting = 0
    --[[
  elseif stage == 4 and setting == 1 then

    init = 0
    duration = 2

    qLArm1 = vector.new(Body.get_larm_position())
    qRArm1 = vector.new(Body.get_rarm_position())
    qLLeg1 = vector.new(Body.get_lleg_position())
    qRLeg1 = vector.new(Body.get_rleg_position())
      qRLeg1[1] = 0
      qRLeg1[6] = 0
    qWaist1 = vector.new(Body.get_waist_position())

    setting = 0

    --]]

  elseif stage == 4 and setting == 1 then

    print("STAGE START!:",stage)
    init = 1
    duration = 5
  
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
    r_trPos[3] = r_trPos[3] - 0.1 -- move arm forward
    r_trPos[2] = r_trPos[2] - util.sign(r_trPos[2])*0.02 -- move closer along with horizontal axis

    pitch_offset = 20*DEG_TO_RAD
    trR = {}
    trR[1] = r_trPos[1]
    trR[2] = r_trPos[2]
    trR[3] = r_trPos[3]
    trR[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trR[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset --pitch
    trR[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll


    r_trPos_init2g = r_trPos2g
    --r_trPos_init2g[3] = r_trPos2g[3]/2
    r_trPos_init = util.mulvec(util.transpose(rpyR_b2g),r_trPos_init2g)

    trR_init = {}
    trR_init[1] = r_trPos_init[1]
    trR_init[2] = r_trPos_init[2]
    trR_init[3] = r_trPos_init[3]
    trR_init[4] = (atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) + currentRArm[4])/2 --yaw
    trR_init[5] = (atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + currentRArm[5])/2 --pitch
    trR_init[6] = (atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) + currentLArm[6])/2 --roll : but about x-axis

    l_trPos2g[3] = l_trPos2g[3] + 0.1
    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    l_trPos[3] = l_trPos[3] + 0.1 -- move arm forward
    l_trPos[2] = l_trPos[2] - util.sign(l_trPos[2])*0.05 -- move closer along with horizontal axis

    pitch_offset = 20*DEG_TO_RAD
    trL = {}
    trL[1] = l_trPos[1]
    trL[2] = l_trPos[2]
    trL[3] = l_trPos[3]
    trL[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trL[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset --pitch
    trL[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll
    --------Left Leg

    stage_h = baseframe_h--+0.012    

    l_hip = Body.get_forward_lleg_origins(qLLeg_current,2)
    l_foot = Body.get_forward_lleg_origins(qLLeg_current,6)

    l_pos_hip = {} -- 3 by 1
      l_pos_hip[1] = l_hip[1]
      l_pos_hip[2] = l_hip[2]
      l_pos_hip[3] = l_hip[3]

    lleg_pos_eff = {}
      lleg_pos_eff[1] = l_foot[1]
      lleg_pos_eff[2] = l_foot[2]
      lleg_pos_eff[3] = l_foot[3]

    l_hip2g  = util.mulvec(rpyR_b2g,l_pos_hip) -- 3 by 1
    lleg_current2g   = util.mulvec(rpyR_b2g,lleg_pos_eff)

    lleg_trPos2g = {}
      lleg_trPos2g[1] = l_hip2g[1]
      lleg_trPos2g[2] = l_hip2g[2]
      lleg_trPos2g[3] = stage_h

    lleg_trPos_init2g = {}
      lleg_trPos_init2g[1] = lleg_current2g[1]
      lleg_trPos_init2g[2] = lleg_current2g[2]
      lleg_trPos_init2g[3] = lleg_current2g[3]+0.01

    lleg_trPos = util.mulvec(util.transpose(rpyR_b2g), lleg_trPos2g)
    lleg_trPos[3] = lleg_trPos[3] - 0.1 -- negative farther than hip joint (vertical body)  util.sign(rleg_trPos[3])*
    lleg_trPos[2] = lleg_trPos[2] -- util.sign(lleg_trPos[2])*0.05
    
    lleg_trPos_init = util.mulvec(util.transpose(rpyR_b2g),lleg_trPos_init2g)
    lleg_trPos_init[3] = lleg_trPos_init[3] + 0.05 -- closer than current right foot joint (vertical body) util.sign(rleg_trPos_init[3])*
    lleg_trPos_init[2] = lleg_trPos_init[2] - util.sign(lleg_trPos_init[2])*0.05

    trLLeg_init = {}
    trLLeg_init[1] = lleg_trPos_init[1]
    trLLeg_init[2] = lleg_trPos_init[2]
    trLLeg_init[3] = lleg_trPos_init[3]
    trLLeg_init[4] = currentLLeg[4] --+ atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]))/2 --yaw
    trLLeg_init[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trLLeg_init[6] = currentLLeg[6]-- + atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) )/2--roll : but about x-axis

    trLLeg = {}
    trLLeg[1] = lleg_trPos[1]
    trLLeg[2] = lleg_trPos[2]
    trLLeg[3] = lleg_trPos[3]
    trLLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trLLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trLLeg[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    rleg_trPos2g[3] = lleg_trPos2g[3]
    rleg_trPos = util.mulvec(util.transpose(rpyR_b2g), rleg_trPos2g)
    rleg_trPos[3] = rleg_trPos[3] - 0.1 -- negative farther than hip joint (vertical body)  util.sign(rleg_trPos[3])*
    rleg_trPos[2] = rleg_trPos[2] -- util.sign(rleg_trPos[2])*0.05

    trRLeg = {}
    trRLeg[1] = rleg_trPos[1]
    trRLeg[2] = rleg_trPos[2]
    trRLeg[3] = rleg_trPos[3]
    trRLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trRLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trRLeg[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll
    --------
    qLLeg1 = vector.new(Body.get_inverse_lleg(trLLeg))
    --qLLeg1 = vector.new(Body.get_lleg_position())
    --  qLLeg1[3] = qLLeg1[3]+util.sign(qLLeg1[3])*fold_ang*DEG_TO_RAD
    --  qLLeg1[4] = qLLeg1[4]+util.sign(qLLeg1[4])*fold_ang*DEG_TO_RAD
    qRLeg1 = vector.new(Body.get_inverse_rleg(trRLeg))
    -- qRLeg1[3] = qRLeg1[3]+util.sign(qRLeg1[3])*fold_ang*DEG_TO_RAD
    -- qRLeg1[4] = qRLeg1[4]+util.sign(qRLeg1[4])*fold_ang*DEG_TO_RAD
      qRLeg1[1] = 0
      qRLeg1[6] = 0
    qWaist1 = vector.new(Body.get_waist_position())
      qWaist1[2] = qWaist1[2] - 0*DEG_TO_RAD
      --qWaist1[1] = -20*DEG_TO_RAD
    qLArm1 = vector.new(Body.get_larm_position())
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    --qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL, qLArm_current[3], qWaist1, qWaist1[1]))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD

    qLArm_init = qLArm1
    qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
      qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD
    --qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
      --qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD
    qLLeg_init = vector.new(Body.get_inverse_lleg(trLLeg_init))
    qRLeg_init = qRLeg1
    qWaist_init = (qWaist0 + qWaist1)/2


    setting = 0

  elseif stage == 5 and setting == 1 then
    init = 0
    duration = 2

    qLArm1 = vector.new(Body.get_larm_position())
    qRArm1 = vector.new(Body.get_rarm_position())
    qLLeg1 = vector.new(Body.get_lleg_position())
    qRLeg1 = vector.new(Body.get_rleg_position())
      qLLeg1[1] = util.sign(qLLeg1[1])*qRLeg1[1]
      qLLeg1[2] = util.sign(qLLeg1[2])*qRLeg1[2]
      qLLeg1[3] = util.sign(qLLeg1[3])*qRLeg1[3]
      qLLeg1[4] = util.sign(qLLeg1[4])*qRLeg1[4]
      qLLeg1[5] = util.sign(qLLeg1[5])*qRLeg1[5]
      qLLeg1[6] = util.sign(qLLeg1[6])*qRLeg1[6]
    qWaist1[1] = 0
    qWaist1[2] = 15*DEG_TO_RAD


    setting = 0



  elseif stage == 6 and setting == 1 then

    init = 1
    duration = 5
  
    --l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
    --l_trPos[3] = l_trPos[3] + 0.1 -- move arm forward
    r_trPos[3] = r_trPos[3] + 0.1

    pitch_offset = 20*DEG_TO_RAD
    --[[
    trL = {}
    trL[1] = l_trPos[1]
    trL[2] = l_trPos[2]
    trL[3] = l_trPos[3]
    trL[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trL[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset --pitch
    trL[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll
]]
    trR = {}
    trR[1] = r_trPos[1]
    trR[2] = r_trPos[2]
    trR[3] = r_trPos[3]
    trR[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trR[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset--pitch
    trR[6] = -atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll


--[[
    l_trPos_init2g = l_trPos2g
    l_trPos_init2g[3] = l_trPos2g[3] + 0.2
    l_trPos_init = util.mulvec(util.transpose(rpyR_b2g),l_trPos_init2g)

    trL_init = {}
    trL_init[1] = l_trPos_init[1]
    trL_init[2] = l_trPos_init[2]
    trL_init[3] = l_trPos_init[3]
    trL_init[4] = (atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) + currentLArm[4])/2 --yaw
    trL_init[5] = (atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + currentLArm[5])/2 --pitch
    trL_init[6] = (atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) + currentLArm[6])/2 --roll : but about x-axis
]]
    r_trPos_init2g = r_trPos2g
    r_trPos_init2g[3] = r_trPos2g[3]/0.2
    r_trPos_init = util.mulvec(util.transpose(rpyR_b2g),r_trPos_init2g)

    trR_init = {}
    trR_init[1] = r_trPos_init[1]
    trR_init[2] = r_trPos_init[2]
    trR_init[3] = r_trPos_init[3]
    trR_init[4] = (atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) + currentRArm[4])/2 --yaw
    trR_init[5] = (atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + currentRArm[5])/2 --pitch
    trR_init[6] = (atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) + currentLArm[6])/2 --roll : but about x-axis


    fold_ang = 15

    qLLeg1 = vector.new(Body.get_lleg_position())
      --qLLeg1[3] = qLLeg1[3]+util.sign(qLLeg1[3])*fold_ang*DEG_TO_RAD
      --qLLeg1[4] = qLLeg1[4]+util.sign(qLLeg1[4])*fold_ang*DEG_TO_RAD
    qRLeg1 = vector.new(Body.get_rleg_position())
      qRLeg1[3] = qRLeg1[3]+util.sign(qRLeg1[3])*fold_ang*DEG_TO_RAD
      qRLeg1[4] = qRLeg1[4]+util.sign(qRLeg1[4])*fold_ang*DEG_TO_RAD
    qWaist1 = vector.new(Body.get_waist_position())
      qWaist1[2] = qWaist1[2] - fold_ang*0.6*DEG_TO_RAD

    --qWaist1[1] = -20*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    --qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL, qLArm_current[3], qWaist1, qWaist1[1]))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD

    qLArm1 = vector.new(Body.get_larm_position())
    --qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    --qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR, qRArm_current[3], qWaist1, qWaist1[1]))

    qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
      qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD
    --qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
      --qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD
    qLArm_init = qLArm1
    qLLeg_init = (qLLeg0 + qLLeg1)/2
    qRLeg_init = (qRLeg0 + qRLeg1)/2
    qWaist_init = (qWaist0 + qWaist1)/2


    setting = 0

  elseif stage == 7 and setting == 1 then
    --*** Stage parameters ***--
    init = 1
    duration = 10
    stage_h = baseframe_h    

    --2) Compute the desired positions for left leg
    l_hip = Body.get_forward_lleg_origins(qLLeg_current,2)
    l_foot = Body.get_forward_lleg_origins(qLLeg_current,6)
    r_foot = Body.get_forward_rleg_origins(qRLeg_current,6)

    l_pos_hip = {} -- 3 by 1
      l_pos_hip[1] = l_hip[1]
      l_pos_hip[2] = l_hip[2]
      l_pos_hip[3] = l_hip[3]

    lleg_pos_eff = {}
      lleg_pos_eff[1] = l_foot[1] -- currentLLeg[1] 
      lleg_pos_eff[2] = l_foot[2] -- currentLLeg[2]
      lleg_pos_eff[3] = l_foot[3] -- currentLLeg[3]
    rleg_pos_eff = {}
      rleg_pos_eff[1] = r_foot[1]
      rleg_pos_eff[2] = r_foot[2]
      rleg_pos_eff[3] = r_foot[3]

    l_hip2g  = util.mulvec(rpyR_b2g,l_pos_hip) -- 3 by 1
    lleg_current2g   = util.mulvec(rpyR_b2g,lleg_pos_eff)
    rleg_current2g   = util.mulvec(rpyR_b2g,rleg_pos_eff)

    lleg_trPos2g = {}
      lleg_trPos2g[1] = l_hip2g[1]
      lleg_trPos2g[2] = l_hip2g[2]
      lleg_trPos2g[3] = stage_h

    rleg_trPos2g = {}
      rleg_trPos2g[1] = rleg_current2g[1]
      rleg_trPos2g[2] = rleg_current2g[2]
      rleg_trPos2g[3] = rleg_current2g[3]

    lleg_trPos_init2g = {}
      lleg_trPos_init2g[1] = lleg_current2g[1]+0.01
      lleg_trPos_init2g[2] = lleg_current2g[2]
      lleg_trPos_init2g[3] = lleg_current2g[3]+0.05

    lleg_trPos = util.mulvec(util.transpose(rpyR_b2g), lleg_trPos2g)
    lleg_trPos_init = util.mulvec(util.transpose(rpyR_b2g),lleg_trPos_init2g)

    trLLeg_init = {}
    trLLeg_init[1] = lleg_trPos_init[1]
    trLLeg_init[2] = lleg_trPos_init[2]
    trLLeg_init[3] = lleg_trPos_init[3]
    trLLeg_init[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) --yaw
    trLLeg_init[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) --pitch
    trLLeg_init[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) --roll : but about x-axis

    trLLeg = {}
    trLLeg[1] = lleg_trPos[1]
    trLLeg[2] = lleg_trPos[2]
    trLLeg[3] = lleg_trPos[3]
    trLLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trLLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trLLeg[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    --3) Compute the desired positions for arms ( adjust heights )

     l_pos_eff = {}
      l_pos_eff[1] = currentLArm[1]
      l_pos_eff[2] = currentLArm[2]
      l_pos_eff[3] = currentLArm[3]

    r_pos_eff = {}
      r_pos_eff[1] = currentRArm[1]
      r_pos_eff[2] = currentRArm[2]
      r_pos_eff[3] = currentRArm[3]

    l_current2g   = util.mulvec(rpyR_b2g,l_pos_eff)
    r_current2g   = util.mulvec(rpyR_b2g,r_pos_eff)

    l_trPos2g = {}
      l_trPos2g[1] = l_current2g[1]
      l_trPos2g[2] = l_current2g[2]
      l_trPos2g[3] = stage_h

    r_trPos2g = {}
      r_trPos2g[1] = r_current2g[1]
      r_trPos2g[2] = r_current2g[2]
      r_trPos2g[3] = stage_h

    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)

    pitch_offset = 20*DEG_TO_RAD
    trL = {}
    trL[1] = l_trPos[1]
    trL[2] = l_trPos[2]
    trL[3] = l_trPos[3]
    trL[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trL[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset --pitch
    trL[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    trR = {}
    trR[1] = r_trPos[1]
    trR[2] = r_trPos[2]
    trR[3] = r_trPos[3]
    trR[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trR[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset--pitch
    trR[6] = -atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    --4) Compute Joint positions 
    
    qLLeg1 = vector.new(Body.get_inverse_lleg(trLLeg))
    qRLeg1 = vector.new(Body.get_rleg_position())
    qWaist1 = vector.new(Body.get_waist_position())
    qWaist1[1] = 20*DEG_TO_RAD

    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL, qLArm_current[3], qWaist1, qWaist1[1]))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR, qRArm_current[3], qWaist1, qWaist1[1]))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD

    --5) Compute Joint positions for Init
    qLArm_init = qLArm1
    qRArm_init = qRArm1

    qLLeg_init = vector.new(Body.get_inverse_lleg(trLLeg_init))
    --qLLeg_init = vector.new(Body.get_lleg_position())
    --qLLeg_init[3] = qLLeg_init[3]- (qLLeg_init[3]/math.abs(qLLeg_init[3]))*15*DEG_TO_RAD
    --qLLeg_init[4] = qLLeg_init[4]+ (qLLeg_init[4]/math.abs(qLLeg_init[4]))*15*DEG_TO_RAD
    qRLeg_init = qRLeg1
    qWaist_init = qWaist1

    setting = 0
--[[
    elseif stage == 4 and setting == 1 then
    --*** Stage parameters ***--
    init = 1
    duration = 10    

    --2) Compute the desired positions 
    l_hip = Body.get_forward_lleg_origins(qLLeg_current,3)
    r_hip = Body.get_forward_rleg_origins(qRLeg_current,4)

    l_pos_hip = {} -- 3 by 1
      l_pos_hip[1] = l_hip[1]
      l_pos_hip[2] = l_hip[2]
      l_pos_hip[3] = l_hip[3]

    r_pos_hip = {} -- 3 by 1
      r_pos_hip[1] = r_hip[1]
      r_pos_hip[2] = r_hip[2]
      r_pos_hip[3] = r_hip[3]

    lleg_pos_eff = {}
      l_pos_eff[1] = currentLLeg[1]
      l_pos_eff[2] = currentLLeg[2]
      l_pos_eff[3] = currentLLeg[3]

    rleg_pos_eff = {}
      r_pos_eff[1] = currentRLeg[1]
      r_pos_eff[2] = currentRLeg[2]
      r_pos_eff[3] = currentRLeg[3]

    l_hip2g  = util.mulvec(rpyR_b2g,l_pos_hip) -- 3 by 1
    r_hip2g  = util.mulvec(rpyR_b2g,r_pos_hip) -- 3 by 1

    lleg_current2g   = util.mulvec(rpyR_b2g,lleg_pos_eff)
    rleg_current2g   = util.mulvec(rpyR_b2g,rleg_pos_eff)

    lleg_trPos2g = {}
      lleg_trPos2g[1] = l_hip2g[1]
      lleg_trPos2g[2] = l_hip2g[2]
      lleg_trPos2g[3] = baseframe_h

    rleg_trPos2g = {}
      rleg_trPos2g[1] = r_hip2g[1]
      rleg_trPos2g[2] = r_hip2g[2]
      rleg_trPos2g[3] = baseframe_h

    lleg_trPos_init2g = {}
      lleg_trPos_init2g[1] = lleg_current2g[1]+0.1
      lleg_trPos_init2g[2] = lleg_current2g[2]
      lleg_trPos_init2g[3] = lleg_current2g[3]+0.3
    
    rleg_trPos_init2g = {}
      rleg_trPos_init2g[1] = rleg_current2g[1]+0.1
      rleg_trPos_init2g[2] = rleg_current2g[2]
      rleg_trPos_init2g[3] = rleg_current2g[3]

    lleg_trPos = util.mulvec(util.transpose(rpyR_b2g), lleg_trPos2g)
    rleg_trPos = util.mulvec(util.transpose(rpyR_b2g), rleg_trPos2g)

    lleg_trPos_init = util.mulvec(util.transpose(rpyR_b2g),lleg_trPos_init2g)

    trLLeg_init = {}
    trLLeg_init[1] = lleg_trPos_init[1]
    trLLeg_init[2] = lleg_trPos_init[2]
    trLLeg_init[3] = lleg_trPos_init[3]
    trLLeg_init[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) --yaw
    trLLeg_init[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) --pitch
    trLLeg_init[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) --roll : but about x-axis

    trLLeg = {}
    trLLeg[1] = lleg_trPos[1]
    trLLeg[2] = lleg_trPos[2]
    trLLeg[3] = lleg_trPos[3]
    trLLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trLLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trLLeg[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    trRLeg = {}
    trRLeg[1] = rleg_trPos[1]
    trRLeg[2] = rleg_trPos[2]
    trRLeg[3] = rleg_trPos[3]
    trRLeg[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trRLeg[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) )--pitch
    trRLeg[6] = -atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    pitch_offset = 12*DEG_TO_RAD
    trL = vector.new(Body.get_larm_position())
    trL[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trL[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset --pitch
    trL[6] = atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    trR = vector.new(Body.get_rarm_position())
    trR[4] = atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3])--yaw
    trR[5] = atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + pitch_offset--pitch
    trR[6] = -atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1])--roll

    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD
    
    qLLeg1 = vector.new(Body.get_inverse_lleg(trLLeg))
    qRLeg1 = vector.new(Body.get_rleg_position())
    qWaist1 = vector.new(Body.get_waist_position())

    -- Init

    qLArm_init = qLArm1
    qRArm_init = qRArm1

    qLLeg_init = vector.new(Body.get_inverse_lleg(trLLeg_init))
    qRLeg_init = qRLeg1
    qWaist_init = qWaist1

    setting = 0
    ]]

  end
  if ori_ctrl_larm == 1 then
    local ori_larm = {}
    ori_larm[1]=currentLArm[6] --roll
    ori_larm[2]=currentLArm[5] --pitch
    ori_larm[3]=currentLArm[4] --yaw

    local R_c2b = {}
    R_c2b[1] = {}
      R_c2b[1][1] = cos(ori_larm[1])*cos(ori_larm[2])
      R_c2b[1][2] = cos(ori_larm[1])*sin(ori_larm[2])*sin(ori_larm[3]) - sin(ori_larm[1])*cos(ori_larm[3])
      R_c2b[1][3] = cos(ori_larm[1])*sin(ori_larm[2])*cos(ori_larm[3]) + sin(ori_larm[1])*sin(ori_larm[3])
    R_c2b[2] = {}
      R_c2b[2][1] = sin(ori_larm[1])*cos(ori_larm[2])
      R_c2b[2][2] = sin(ori_larm[1])*sin(ori_larm[2])*sin(ori_larm[3]) + cos(ori_larm[1])*cos(ori_larm[3])
      R_c2b[2][3] = sin(ori_larm[1])*sin(ori_larm[2])*cos(ori_larm[3]) - cos(ori_larm[1])*sin(ori_larm[3]) 
    R_c2b[3] = {}
      R_c2b[3][1] = -sin(ori_larm[2])
      R_c2b[3][2] = cos(ori_larm[2])*sin(ori_larm[3])
      R_c2b[3][3] = cos(ori_larm[2])*cos(ori_larm[3])

    local R_c2g = util.mul(R_c2b,rpyR_b2g)

    -- PITCH
    qLArm1[6] = qLArm1[6] + (rpy[2] - atan2(R_c2g[3][1],sqrt( R_c2g[3][2]^2 + R_c2g[3][3]^2) ))
    -- ROLL
    qLArm1[7] = qLArm1[7] + (rpy[2] - atan2(R_c2g[2][1],R_c2g[1][1]))

  end

  if stage>0 and stage < (stagemax+1) then    
    local ph = math.max(0,math.min(1, t_diff/duration))
    
    if init == 1 then -- When there exist an initial state
      ph = ph*2
      if t_diff < 0.5*duration then
        Body.set_larm_command_position((1-ph)*qLArm0 + ph * qLArm_init)    
        Body.set_rarm_command_position((1-ph)*qRArm0 + ph * qRArm_init)
        Body.set_lleg_command_position((1-ph)*qLLeg0 + ph * qLLeg_init)
        Body.set_rleg_command_position((1-ph)*qRLeg0 + ph * qRLeg_init)
        Body.set_waist_command_position((1-ph)*qWaist0 + ph * qWaist_init)
      else
        ph = ph-1
        Body.set_larm_command_position((1-ph)*qLArm_init + ph * qLArm1)    
        Body.set_rarm_command_position((1-ph)*qRArm_init + ph * qRArm1)
        Body.set_lleg_command_position((1-ph)*qLLeg_init + ph * qLLeg1)
        Body.set_rleg_command_position((1-ph)*qRLeg_init + ph * qRLeg1)
        Body.set_waist_command_position((1-ph)*qWaist_init + ph * qWaist1)
      end
    else
      Body.set_larm_command_position((1-ph)*qLArm0 + ph * qLArm1)    
      Body.set_rarm_command_position((1-ph)*qRArm0 + ph * qRArm1)
      Body.set_lleg_command_position((1-ph)*qLLeg0 + ph * qLLeg1)
      Body.set_rleg_command_position((1-ph)*qRLeg0 + ph * qRLeg1)
      Body.set_waist_command_position((1-ph)*qWaist0 + ph * qWaist1)
    end
    print(t_diff)
    print(duration)

  else
    hcm.set_state_proceed(0)
    print("DONE")
    return "done"
  end
  -- Go to next stage
  if stage == 0 or (t_diff>duration and hcm.get_state_proceed()==1) then
    if stage > 0 and stop == 1 then 
      hcm.set_state_proceed(0) 
    end
    setting = 1
    stage = stage + 1
    t_start=t

    qLArm0 = qLArm1
    qRArm0 = qRArm1
    qLLeg0 = qLLeg1
    qRLeg0 = qRLeg1
    qWaist0 = qWaist1
    
    unix.usleep(2000000)
  end
end -- walk.update

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
  hcm.set_motion_headangle({0,0*math.pi/180})
end

return state

