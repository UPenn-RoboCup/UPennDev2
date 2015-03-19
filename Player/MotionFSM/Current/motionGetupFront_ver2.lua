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

 -- Constants
  local pitch_offset = 20*DEG_TO_RAD

  local knee_offset = 0.02
  local stagemax = 3
  local stop = 0

  local neckOffsetZ = .165+.161; --Webots value
  local neckOffsetX = 0;

  local shoulderOffsetX = 0;    
  local shoulderOffsetY = 0.234;
  local shoulderOffsetZ = 0.165;

  local upperArmLength = .246;
  local elbowOffsetX =   .030; 

  local lowerArmLength = .186; --Default 7DOF arm
  local lowerArmLength = .250; --LONGARM model

  local handOffsetX = 0.245; --Measured from robot
  local handOffsetY = 0.035; --Measured from robot
  local handOffsetZ = 0;

  local hipOffsetX = 0;
  local hipOffsetY = 0.072;
  local hipOffsetZ = 0.282; 

  local thighLength = 0.30;
  local tibiaLength = 0.30;
  local kneeOffsetX = 0.03;

  local footHeight = 0.118; -- Webots value
  local footToeX = 0.130; --from ankle to toe
  local footHeelX = 0.130; --from ankle to heel

  local gain1 = 0.01
  local gain2 = 0.03
  local gain3 = 0.08

  local Kp = 0.35
  local Kd = -0.1


  -- Initialize
  local mov = {}
  local ori_ctrl_larm = 0
  local ori_ctrl_rarm = 0
  local ori_ctrl_lleg = 0
  local ori_ctrl_rleg = 0
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
  
  -- Check wheter the robot was fallen down or not
  if math.abs(rpy[1])<45*math.pi/180 and math.abs(rpy[2])<45*math.pi/180 then      
    return "done"
  end
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

  rpy_target = vector.new{0,0,0}
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

  --[[ Real Time data ]]----------------------------------------------------------------------------

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

  l_shoulder = Body.get_forward_larm_origins(qLArm_current,2)
  r_shoulder = Body.get_forward_rarm_origins(qRArm_current,2)
  l_hip      = Body.get_forward_lleg_origins(qLLeg_current,2)
  r_hip      = Body.get_forward_rleg_origins(qRLeg_current,2)
    l_hip_posg     = util.mulvec(rpyR_b2g,vector.slice(l_hip,1,3))
    r_hip_posg     = util.mulvec(rpyR_b2g,vector.slice(r_hip,1,3))
  l_knee     = Body.get_forward_lleg_origins(qLLeg_current,4)
  r_knee     = Body.get_forward_rleg_origins(qRLeg_current,4)
    l_knee_posg = util.mulvec(rpyR_b2g,vector.slice(l_knee,1,3))
    r_knee_posg = util.mulvec(rpyR_b2g,vector.slice(r_knee,1,3))
  l_foot     = Body.get_forward_lleg_origins(qLLeg_current,6)
  r_foot     = Body.get_forward_rleg_origins(qRLeg_current,6)
    l_foot_posg = util.mulvec(rpyR_b2g,vector.slice(l_foot,1,3))
    r_foot_posg = util.mulvec(rpyR_b2g,vector.slice(r_foot,1,3))

  lleg_current2g   = util.mulvec(rpyR_b2g,vector.slice(currentLLeg,1,3))
  rleg_current2g   = util.mulvec(rpyR_b2g,vector.slice(currentRLeg,1,3))
  

--[[ Find the lowest joint ]]---------------------------------------------------------------------------------------------

  local numJoint = vector.new{7,7,6,6}

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

--[[ stages ]]--------------------------------------------------------------------------------------------------
--[[ DESCRIPTION 
/* 
  moving : final target positions are assigned once
  support : keep track its position w.r.t ground
  free : keep track its position w.r.t body

  1) Put two hands next to the shoulders
    moving : larm, rarm
    support :torso, knees, foot
    free : x

  2) Push up
    moving : larm, rarm
    support : knees, foot
    free : torso

  3) Pull rknee towards CM (knee(x,y,z) = hip(x,y,z-l_thigh))
    moving : upper rleg, waist_roll(=l_hip/(l_thigh-sqrt(l_thigh^2 - l_k2h^2))
    support : larm, rarm*, lleg
    free : rfoot, waist_pitch

  4) Pull lknee towards CM
    moving : upper lleg, waist_roll(0)
    support : larm, rarm*, rleg
    free : lfoot, waist_pitch

  5) Pull rfoot closer but behind hip position
   moving : upper rleg, waist_roll
   support : larm, rarm*, lleg
   free : rfoot, waist_pitch

  6) Pull two arms
    moving : larm, rarm, waist_pitch
    support : one of the arms not moving, two knees
    free : 

  7) Move CM and make the rfoot support the body (th = rfoot pitch w.r.t ground)
   moving : waist_pitch, lhip 
   support : lknee, lfoot 
   free : two arms, rleg

  8) Slightly stand up move lleg and put lfoot next to rfoot
    moving : rknee, rhip, lleg
    support : two arms, rfoot
/*
--]]
  
--[[
1) Put two hands next to the shoulders
    moving : larm, rarm
    support :torso, knees, foot
    free : x
--]]
  
  if stage == 1 then
     --*** Stage parameters ***--
    init = 1
    duration = 3
    mov = vector.new{1,1,1,1,0}
    ori_ctrl_larm = 0
    ori_ctrl_rarm = 0
    
    if setting == 1 then -- : Set the target positions and orientations for moving parts
      --(a) Support Parts for the current STAGE
      supportIdx = {}
      j=0
      for i=1,jointCount do
        if math.abs(j_heights[i]-baseframe_h) < 0.05 then
          supportIdx[j] = i
          j = j+1
        end
      end
      -- If : support parts include left or/and right hand -> move up first

      --(b) Compute the desired positions 

      l_pos_sh = vector.slice(l_shoulder,1,3)
      r_pos_sh = vector.slice(r_shoulder,1,3)

      l_pos_eff = vector.slice(currentLArm,1,3)
      r_pos_eff = vector.slice(currentRArm,1,3)

      l_shoulder2g  = util.mulvec(rpyR_b2g,l_pos_sh) -- 3 by 1
      r_shoulder2g  = util.mulvec(rpyR_b2g,r_pos_sh) -- 3 by 1

      l_current2g   = util.mulvec(rpyR_b2g,l_pos_eff)
      r_current2g   = util.mulvec(rpyR_b2g,r_pos_eff)

      l_trPos2g = l_shoulder2g
        l_trPos2g[3] = baseframe_h-0.15

      r_trPos2g = r_shoulder2g
        r_trPos2g[3] = baseframe_h-0.15

      l_trPos_init2g = l_current2g
        l_trPos_init2g[3] = l_trPos_init2g[3] + 0.2
      
      r_trPos_init2g = r_current2g
        r_trPos_init2g[3] = r_trPos_init2g[3] + 0.2 

      l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
        l_trPos[3] = l_trPos[3]+0.25
        l_trPos[2] = l_trPos[2]+util.sign(l_trPos[2])*0.01
      r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
        r_trPos[3] = r_trPos[3]+0.25
        r_trPos[2] = r_trPos[2]+util.sign(r_trPos[2])*0.01

      l_trPos_init = util.mulvec(util.transpose(rpyR_b2g),l_trPos_init2g)
        l_trPos_init[3] = (l_trPos[3] + l_pos_eff[3])/2
        l_trPos_init[2] = (l_trPos[2] + l_pos_eff[2])/2
      r_trPos_init = util.mulvec(util.transpose(rpyR_b2g),r_trPos_init2g)
        r_trPos_init[3] = (r_trPos[3] + r_pos_eff[3])/2
        r_trPos_init[2] = (r_trPos[2] + r_pos_eff[2])/2

      setting = 0

    end
    trL_init = util.concat(l_trPos_init, util.ypr(rpyR_tr2b,1))      
    trR_init = util.concat(r_trPos_init, util.ypr(rpyR_tr2b,-1))
    --[[
    trL_init = {}
    trL_init[1] = l_trPos_init[1]
    trL_init[2] = l_trPos_init[2]
    trL_init[3] = l_trPos_init[3]
    trL_init[4] = (atan2(rpyR_tr2b[3][2],rpyR_tr2b[3][3]) + currentLArm[4])/2 --yaw
    trL_init[5] = (atan2(-rpyR_tr2b[3][1],sqrt( rpyR_tr2b[3][2]^2 + rpyR_tr2b[3][3]^2) ) + currentLArm[5])/2 --pitch
    trL_init[6] = (atan2(rpyR_tr2b[2][1],rpyR_tr2b[1][1]) + currentLArm[6])/2 --roll : but about x-axis
    ]]
    trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1))
    trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1))
    
    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD
    
    qLArm_init = vector.new(Body.get_inverse_larm(qLArm_current, trL_init))
    qLArm_init[7] = qLArm_init[7] + 90*DEG_TO_RAD
    qRArm_init = vector.new(Body.get_inverse_rarm(qRArm_current, trR_init))
    qRArm_init[7] = qRArm_init[7] - 90*DEG_TO_RAD


    qLLeg1 = vector.new(Body.get_lleg_position())
    qRLeg1 = vector.new(Body.get_rleg_position())
    qWaist1 = vector.new(Body.get_waist_position())

    --qWaist1[2] = qWaist1[2] + (baseframe_h - math.min(l_knee_posg[3],r_knee_posg[3]) + knee_offset) * gain1
    qLLeg1[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((baseframe_h - l_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (lleg_current2g[3] - footToeX))*gain1)
    qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (baseframe_h - l_knee_posg[3] + knee_offset) * gain1
    qRLeg1[4] = qRLeg1[4] - util.sign(qRLeg1[4]) * ((baseframe_h - r_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (rleg_current2g[3] - footToeX))*gain1)
    qRLeg1[3] = qRLeg1[3] - util.sign(qRLeg1[3]) * (baseframe_h - r_knee_posg[3] + knee_offset) * gain1
    
    qLLeg_init = qLLeg1
    qRLeg_init = qRLeg1
    qWaist_init = qWaist1

  elseif stage == 2 then
    --*** Stage parameters ***--
    init = 0
    duration = 2
    mov = vector.new{1,1,0,0,0}

    if setting == 1 then
      
      l_trPos2g[3] = baseframe_h-0.25
      r_trPos2g[3] = baseframe_h-0.25

      l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
        l_trPos[3] = l_trPos[3]+0.25
        l_trPos[2] = l_trPos[2]+util.sign(l_trPos[2])*0.01
      r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
        r_trPos[3] = r_trPos[3]+0.25
        r_trPos[2] = r_trPos[2]+util.sign(r_trPos[2])*0.01

      trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1))
      trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1))
      
      setting = 0
    end

    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD
    
    qLLeg1 = vector.new(Body.get_lleg_position())
    qRLeg1 = vector.new(Body.get_rleg_position())
    qWaist1 = vector.new(Body.get_waist_position())

    qWaist1[2] = qWaist1[2] + (baseframe_h - math.min(l_knee_posg[3],r_knee_posg[3]) + knee_offset) * gain1
    qLLeg1[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((baseframe_h - l_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (lleg_current2g[3]-footToeX))*gain1)
    qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (baseframe_h - l_knee_posg[3] + knee_offset) * gain1
    qRLeg1[4] = qRLeg1[4] - util.sign(qRLeg1[4]) * ((baseframe_h - r_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (rleg_current2g[3]-footToeX))*gain1)
    qRLeg1[3] = qRLeg1[3] - util.sign(qRLeg1[3]) * (baseframe_h - r_knee_posg[3] + knee_offset) * gain1

--[[
 3) Pull rknee towards CM (knee(x,y,z) = hip(x,y,z-l_thigh))
    moving : upper rleg, waist_roll(=l_hip/(l_thigh-sqrt(l_thigh^2 - l_k2h^2))
    support : larm, rarm*, lleg
    free : rfoot, waist_pitch
]]
  
  elseif stage == 3 then
    init = 1
    duration = 10

    mov = vector.new{0,0,0,0,1}

    if setting == 1 then
      print("STAGE START setting!:",stage)
      setting = 0
      qRLeg_preInit = qRLeg0
      qRLeg_pre     = qRLeg0
      qWaist1 = vector.new(Body.get_waist_position())
        qWaist1[2] = qWaist1[2] - 15*DEG_TO_RAD

    end
    
    l_trPos2g[3] = baseframe_h -- - hipOffsetY*sin(qWaist_current[1])/2
    r_trPos2g[3] = baseframe_h -0.12*t_diff/duration-- + hipOffsetY*sin(qWaist_current[1])/2

    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
    trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1))
    trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1))


    r_knee_posg[1] = r_knee_posg[1] + tibiaLength   
    h_diff = thighLength * ( sin(math.abs(qWaist_current[2]) + math.abs(qLLeg_current[3])) - sin(math.abs(qWaist_current[2]) + math.abs(qRLeg_current[3])) ) -- negative for h_left < h_right
    tilt = h_diff * 0.1
    print("h_diff", h_diff)

    qRLeg1 = vector.new(Body.get_rleg_position())
      qRLeg1[1] = qRLeg1[1] - qRLeg1[1]*t_diff/duration--Kp * (0*DEG_TO_RAD - qRLeg1[1]) + Kd * (qRLeg1[1] - qRLeg_pre[1])
      print("qRLeg1[1] command ", qRLeg1[1])
      qRLeg1[3] = qRLeg1[3] + util.sign(qRLeg1[3]) * (r_hip_posg[1] - r_knee_posg[1]) * gain3
      qRLeg1[4] = qRLeg1[4] - util.sign(qRLeg1[4]) * ((baseframe_h - r_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (rleg_current2g[3]-footToeX)) * gain2)
      --qRLeg_pre = qRLeg1
    qLLeg1 = vector.new(Body.get_lleg_position())
      --qLLeg1[1] = -qWaist_current[1]
      --[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((baseframe_h - l_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (lleg_current2g[3]-footToeX)) * gain1)
      --qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (baseframe_h - l_knee_posg[3] + knee_offset) * gain1

    qRLeg_init = qRLeg1
      qRLeg_init[1] = (-20*DEG_TO_RAD - qRLeg0[1])*t_diff/duration--Kp * (-20*DEG_TO_RAD - qRLeg_init[1]) + Kd * (qRLeg_init[1] - qRLeg_pre[1])

      if t_diff < 0.5*duration then
        qRLeg_pre = qRLeg_init
      end

    qLLeg_init = qLLeg1

    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD
    qLArm_init = qLArm1
    qRArm_init = qRArm1
    
--[[
  elseif stage == 3 then
    init = 1
    duration = 10
    mov = vector.new{0,0,0,0,0}

    if setting == 1 then
      print("STAGE START setting!:",stage)
      
      gain3 = 0.08

      --(a) support part setting
      l_trPos2g[3] = baseframe_h
      r_trPos2g[3] = baseframe_h

      --(b) Constant moving part setting
      desired = vector.new{
       - 10*DEG_TO_RAD,
       - 5*DEG_TO_RAD,
       util.sign(qRLeg0[3])*5*DEG_TO_RAD,
       0,
       - 0.3 }

      lleg_pre = qLLeg0[1]
      waist_pre = qWaist0[1]
      prevRLeg_i = qRLeg0
      prevRLeg = util.concat(vector.slice(desired,1,3),vector.new{qRLeg0[4],-0.3,qRLeg0[6]})

     

      setting = 0
      
    end

    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
    trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1))
    trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1))

    tilt = -((rleg_current2g[3]-tibiaLength*cos(qRLeg_current[4])) - (lleg_current2g[3] - tibiaLength*cos(qLLeg_current[4])))

    r_knee_posg[1] = r_knee_posg[1] + tibiaLength 

    print("tilt",tilt)
    
    qRLeg1 = vector.new(Body.get_rleg_position())
      qRLeg1[1] = Kp*(tilt - qRLeg1[1]) + Kd * (qRLeg1[1] - prevRLeg[1])
      qRLeg1[2] = Kp*(0 - qRLeg1[2]) + Kd * (qRLeg1[2] - prevRLeg[2])
      qRLeg1[3] = qRLeg1[3] + util.sign(qRLeg1[3])*(r_hip_posg[1] - r_knee_posg[1]) * gain3
      qRLeg1[4] = qRLeg1[4] - util.sign(qRLeg1[4]) * ((baseframe_h - r_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (rleg_current2g[3]-footToeX)) * gain1)
      prevRLeg = qRLeg1

    qLLeg1 = vector.new(Body.get_lleg_position())
      qLLeg1[1] = Kp*(tilt - qLLeg1[1]) + Kd * (qLLeg1[1] - lleg_pre)
        lleg_pre = qLLeg1[1]
      qLLeg1[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((baseframe_h - l_knee_posg[3] + knee_offset) * gain2/2 - (baseframe_h - (lleg_current2g[3]-footToeX))*gain1)
      qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (baseframe_h - l_knee_posg[3] + knee_offset) * gain1/2
    
    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD
    qWaist1 = vector.new(Body.get_waist_position())
      qWaist1[1] = Kp * (2*tilt - qWaist1[1]) + Kd * (qWaist1[1] - waist_pre)
      waist_pre = qWaist1[1]   

    qRLeg_init = vector.new(qRLeg_current)
    qRLeg_init[1] = Kp * (desired[1] - qRLeg_init[1]) + Kd * (qRLeg_init[1] - prevRLeg_i[1])
    qRLeg_init[2] = Kp * (desired[2] - qRLeg_init[2]) + Kd * (qRLeg_init[1] - prevRLeg_i[2])
    qRLeg_init[3] = Kp * (desired[3] - qRLeg_init[3]) + Kd * (qRLeg_init[1] - prevRLeg_i[3])
    qRLeg_init[5] = Kp * (desired[5] - qRLeg_init[5]) + Kd * (qRLeg_init[1] - prevRLeg_i[5])
    prevRLeg_i = qRLeg_init

    qLLeg_init = qLLeg_current
    qLArm_init = qLArm1
    qRArm_init = qRArm1
    qWaist_init = qWaist_current
  ]]
  elseif stage == 5 then
    print("STAGE START!:",stage)
    init = 1
    duration = 5

    if setting == 1 then

      qWaist_init = vector.new(Body.get_waist_position())
      qWaist1 = qWaist_init
        qWaist_init[1] = -20*DEG_TO_RAD
        qWaist1[1] = -10*DEG_TO_RAD

      l_trPos2g[3] = baseframe_h + shoulderOffsetY * sin(20*DEG_TO_RAD)
      r_trPos2g[3] = baseframe_h - shoulderOffsetY * sin(20*DEG_TO_RAD)

      setting = 0
    end

    tilt = -((rleg_current2g[3]-tibiaLength*math.abs(sin(qRLeg_current[4]))) - (lleg_current2g[3] - tibiaLength*math.abs(sin(qLLeg_current[4]))) )*0.5
    print("tilt",tilt)

    l_trPos = util.mulvec(util.transpose(rpyR_b2g), l_trPos2g)
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
    trL = util.concat(l_trPos, util.ypr(rpyR_tr2b,1))
    trR = util.concat(r_trPos, util.ypr(rpyR_tr2b,-1))

    qRLeg1 = vector.new(Body.get_rleg_position())
      qRLeg1[1] = qRLeg1[1] + qWaist_current[1]
      qRLeg1[3] = qRLeg1[3] + util.sign(qRLeg1[3]) * (r_hip_posg[1] - rleg_current2g[1]) * gain3
      qRLeg1[4] = qRLeg1[4] - util.sign(qRLeg1[4]) * ((baseframe_h - r_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (rleg_current2g[3]-footToeX)) * gain2)
    qRLeg_init = qRLeg1
    qLLeg1 = vector.new(Body.get_lleg_position())
      qLLeg1[1] = qLLeg1[1] + qWaist_current[1]
      qLLeg1[4] = qLLeg1[4] - util.sign(qLLeg1[4]) * ((baseframe_h - l_knee_posg[3] + knee_offset) * gain2 - (baseframe_h - (lleg_current2g[3]-footToeX)) * gain1)
      qLLeg1[3] = qLLeg1[3] - util.sign(qLLeg1[3]) * (baseframe_h - l_knee_posg[3] + knee_offset) * gain1
    qLLeg_init = qLLeg1

    qLArm1 = vector.new(Body.get_inverse_larm(qLArm_current, trL, qLArm_current[3], qWaist1, qWaist1[1]))
    qLArm1[7] = qLArm1[7] + 90*DEG_TO_RAD
    qRArm1 = vector.new(Body.get_inverse_rarm(qRArm_current, trR, qRArm_current[3], qWaist1, qWaist1[1]))
    qRArm1[7] = qRArm1[7] - 90*DEG_TO_RAD












  
    r_trPos = util.mulvec(util.transpose(rpyR_b2g), r_trPos2g)
    r_trPos[3] = r_trPos[3] - 0.1 -- move arm forward
    r_trPos[2] = r_trPos[2] - util.sign(r_trPos[2])*0.02 -- move closer along with horizontal axis

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
    trL = vector.new(Body.ge`t_larm_position())
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
    ori_larm[1] = currentLArm[6] --roll
    ori_larm[2] = currentLArm[5] --pitch
    ori_larm[3] = currentLArm[4] --yaw

    local R_c2b_l = {}
    R_c2b_l[1] = {}
      R_c2b_l[1][1] = cos(ori_larm[1])*cos(ori_larm[2])
      R_c2b_l[1][2] = cos(ori_larm[1])*sin(ori_larm[2])*sin(ori_larm[3]) - sin(ori_larm[1])*cos(ori_larm[3])
      R_c2b_l[1][3] = cos(ori_larm[1])*sin(ori_larm[2])*cos(ori_larm[3]) + sin(ori_larm[1])*sin(ori_larm[3])
    R_c2b_l[2] = {}
      R_c2b_l[2][1] = sin(ori_larm[1])*cos(ori_larm[2])
      R_c2b_l[2][2] = sin(ori_larm[1])*sin(ori_larm[2])*sin(ori_larm[3]) + cos(ori_larm[1])*cos(ori_larm[3])
      R_c2b_l[2][3] = sin(ori_larm[1])*sin(ori_larm[2])*cos(ori_larm[3]) - cos(ori_larm[1])*sin(ori_larm[3]) 
    R_c2b_l[3] = {}
      R_c2b_l[3][1] = -sin(ori_larm[2])
      R_c2b_l[3][2] = cos(ori_larm[2])*sin(ori_larm[3])
      R_c2b_l[3][3] = cos(ori_larm[2])*cos(ori_larm[3])

    local R_c2b_l = util.mul(R_c2b_l,rpyR_b2g)

    -- PITCH
    qLArm1[6] = qLArm1[6] + (rpy[2] - atan2(R_c2b_l[3][1],sqrt( R_c2b_l[3][2]^2 + R_c2b_l[3][3]^2) ))
    -- ROLL
    qLArm1[7] = qLArm1[7] + (rpy[1] - atan2(R_c2b_l[2][1],R_c2b_l[1][1]))

  end

  if ori_ctrl_rarm == 1 then
    local ori_rarm = {}
    ori_rarm[1] = currentLArm[6] --roll
    ori_rarm[2] = currentLArm[5] --pitch
    ori_rarm[3] = currentLArm[4] --yaw

    local R_c2b_r = {}
    R_c2b_r[1] = {}
      R_c2b_r[1][1] = cos(ori_rarm[1])*cos(ori_rarm[2])
      R_c2b_r[1][2] = cos(ori_rarm[1])*sin(ori_rarm[2])*sin(ori_rarm[3]) - sin(ori_rarm[1])*cos(ori_rarm[3])
      R_c2b_r[1][3] = cos(ori_rarm[1])*sin(ori_rarm[2])*cos(ori_rarm[3]) + sin(ori_rarm[1])*sin(ori_rarm[3])
    R_c2b_r[2] = {}
      R_c2b_r[2][1] = sin(ori_rarm[1])*cos(ori_rarm[2])
      R_c2b_r[2][2] = sin(ori_rarm[1])*sin(ori_rarm[2])*sin(ori_rarm[3]) + cos(ori_rarm[1])*cos(ori_rarm[3])
      R_c2b_r[2][3] = sin(ori_rarm[1])*sin(ori_rarm[2])*cos(ori_rarm[3]) - cos(ori_rarm[1])*sin(ori_rarm[3]) 
    R_c2b_r[3] = {}
      R_c2b_r[3][1] = -sin(ori_rarm[2])
      R_c2b_r[3][2] = cos(ori_rarm[2])*sin(ori_rarm[3])
      R_c2b_r[3][3] = cos(ori_rarm[2])*cos(ori_rarm[3])

    local R_c2b_r = util.mul(R_c2b_r,rpyR_b2g)

    -- PITCH
    qRArm1[6] = qRArm1[6] + (rpy[2] - atan2(R_c2b_r[3][1],sqrt( R_c2b_r[3][2]^2 + R_c2b_r[3][3]^2) ))
    -- ROLL
    qRArm1[7] = qRArm1[7] + (rpy[1] + atan2(R_c2b_r[2][1],R_c2b_r[1][1]))

  end



  if stage>0 and stage < (stagemax+1) then    
    local ph = math.max(0,math.min(1, t_diff/duration))
    
    if init == 1 then -- When there exist an initial state
      ph = ph*2
      if t_diff < 0.5*duration then
        Body.set_larm_command_position(mov[1]*((1-ph)*qLArm0 + ph * qLArm_init) + (1-mov[1])*qLArm_init)    
        Body.set_rarm_command_position(mov[2]*((1-ph)*qRArm0 + ph * qRArm_init) + (1-mov[2])*qRArm_init)
        Body.set_lleg_command_position(mov[3]*((1-ph)*qLLeg0 + ph * qLLeg_init) + (1-mov[3])*qLLeg_init)
        Body.set_rleg_command_position(mov[4]*((1-ph)*qRLeg0 + ph * qRLeg_init) + (1-mov[4])*qRLeg_init)
        Body.set_waist_command_position(mov[5]*((1-ph)*qWaist0 + ph * qWaist_init) + (1-mov[5])*qWaist_init)
      else
        ph = ph-1
        Body.set_larm_command_position(mov[1]*((1-ph)*qLArm_init + ph * qLArm1) + (1-mov[1])*qLArm1)    
        Body.set_rarm_command_position(mov[2]*((1-ph)*qRArm_init + ph * qRArm1) + (1-mov[2])*qRArm1)    
        Body.set_lleg_command_position(mov[3]*((1-ph)*qLLeg_init + ph * qLLeg1) + (1-mov[3])*qLLeg1)    
        Body.set_rleg_command_position(mov[4]*((1-ph)*qRLeg_init + ph * qRLeg1) + (1-mov[4])*qRLeg1)    
        Body.set_waist_command_position(mov[5]*((1-ph)*qWaist_init + ph * qWaist1) + (1-mov[5])*qWaist_init)    
      end
    else
      Body.set_larm_command_position(mov[1]*((1-ph)*qLArm0 + ph * qLArm1) + (1-mov[1])*qLArm1)    
      Body.set_rarm_command_position(mov[2]*((1-ph)*qRArm0 + ph * qRArm1) + (1-mov[2])*qRArm1)    
      Body.set_lleg_command_position(mov[3]*((1-ph)*qLLeg0 + ph * qLLeg1) + (1-mov[3])*qLLeg1)    
      Body.set_rleg_command_position(mov[4]*((1-ph)*qRLeg0 + ph * qRLeg1) + (1-mov[4])*qRLeg1)    
      Body.set_waist_command_position(mov[5]*((1-ph)*qWaist0 + ph * qWaist1) + (1-mov[5])*qWaist_init) 
    end
    print(t_diff)
    print(duration)

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
  else
    hcm.set_state_proceed(0)
    print("DONE")
    return "done"
  end
end -- walk.update

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
  hcm.set_motion_headangle({0,0*math.pi/180})
end

return state

