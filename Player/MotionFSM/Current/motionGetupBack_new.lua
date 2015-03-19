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
local movearm = require'movearm'
--local webot = require'webot'

require'mcm'
require'hcm'

local simple_ipc = require'simple_ipc'
local head_ch   = simple_ipc.new_publisher('HeadFSM!')


------- New Code starts from HERE ----------
-- Keep track of important times


local stage
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
  
  qLArm1 = vector.new(Body.get_larm_command_position())  
  qRArm1 = vector.new(Body.get_rarm_command_position())  
  qLLeg1 = vector.new(Body.get_lleg_command_position())  
  qRLeg1 = vector.new(Body.get_rleg_command_position())  
  qWaist1 = vector.new(Body.get_waist_command_position())  

--print(unpack(vector.new(qLArm1)/DEG_TO_RAD))

  stage = 0  
  hcm.set_state_proceed(1)
  head_ch:send'teleop'
  body_ch:send'init'
end



function state.update()
  -- Get the time of update
  --local torque=webot.wb_motor_get_torque_feedback()
  local t = Body.get_time()
  local t_diff = t - t_start


  local rpy = Body.get_rpy()
  
  -- Check wheter the robot was fallen down or not
  if math.abs(rpy[1])<45*math.pi/180 and math.abs(rpy[2])<45*math.pi/180 then      
    return 'done'
  end
  
  -- Get Position vectors of all joints

  qLArm_initial   = Body.get_larm_position();
  qRArm_initial   = Body.get_rarm_position();
  qLLeg_initial   = Body.get_lleg_position();
  qRLeg_initial   = Body.get_rleg_position();
  qWaist_initial  = Body.get_waist_position();
  qHead_initial   = Body.get_head_position();

  -- Find the lowest joint and compute base frame height
  rpyR ={}
    rpyR[1] = {}
      rpyR[1][1] = math.cos(rpy[1])*math.cos(rpy[2])
      rpyR[1][2] = math.cos(rpy[1])*math.sin(rpy[2])*math.sin(rpy[3]) - math.sin(rpy[1])*math.cos(rpy[3])
      rpyR[1][3] = math.cos(rpy[1])*math.sin(rpy[2])*math.cos(rpy[3]) + math.sin(rpy[1])*math.sin(rpy[3])
    rpyR[2] = {}
      rpyR[2][1] = math.sin(rpy[1])*math.cos(rpy[2])
      rpyR[2][2] = math.sin(rpy[1])*math.sin(rpy[2])*math.sin(rpy[3]) + math.cos(rpy[1])*math.cos(rpy[3])
      rpyR[2][3] = math.sin(rpy[1])*math.sin(rpy[2])*math.cos(rpy[3]) - math.cos(rpy[1])*math.sin(rpy[3]) 
    rpyR[3] = {}
      rpyR[3][1] = -math.sin(rpy[2])
      rpyR[3][2] = math.cos(rpy[2])*math.sin(rpy[3])
      rpyR[3][3] = math.cos(rpy[2])*math.cos(rpy[3])

  numJoint = {};
  numJoint[1] = 7
  numJoint[2] = 7
  numJoint[3] = 6
  numJoint[4] = 6
  numJoint[5] = 2
  numJoint[6] = 2

  j_heights = {}
  local merong = 0
  
  for i=1,numJoint[1] do
      or_pos = Body.get_forward_larm_origins(qLArm_initial,i)
      j_heights[merong+i] = rpyR[3][1]*or_pos[1] + rpyR[3][2]*or_pos[2] + rpyR[3][3]*or_pos[3]
  end
  merong = merong + numJoint[1]

  for i=1,numJoint[2] do
      or_pos = Body.get_forward_rarm_origins(qRArm_initial,i)
      j_heights[merong+i] = rpyR[3][1]*or_pos[1] + rpyR[3][2]*or_pos[2] + rpyR[3][3]*or_pos[3]
  end
  merong = merong + numJoint[2]

  for i=1,numJoint[3] do
      or_pos = Body.get_forward_lleg_origins(qLLeg_initial,i)
      j_heights[merong+i] = rpyR[3][1]*or_pos[1] + rpyR[3][2]*or_pos[2] + rpyR[3][3]*or_pos[3]
  end
  merong = merong + numJoint[3]

  for i=1,numJoint[4] do
      or_pos = Body.get_forward_rleg_origins(qRLeg_initial,i)
      j_heights[merong+i] = rpyR[3][1]*or_pos[1] + rpyR[3][2]*or_pos[2] + rpyR[3][3]*or_pos[3]
  end
  --merong = merong + numJoint[4]

  --[[
  for i=1,numJoint[j] do
      or_pos = Body.get_forward_waist_origins(qLArm_initial,i)
      j_heights[merong+i] = rpyR[1][3]*or_pos[1] + rpyR[2][3]*or_pos[2] + rpyR[3][3]*or_pos[3]
  end
  for i=1,numJoint[j] do
      or_pos = Body.get_forward_head_origins(qLArm_initial,i)
      j_heights[merong+i] = rpyR[1][3]*or_pos[1] + rpyR[2][3]*or_pos[2] + rpyR[3][3]*or_pos[3]
  end
--]]
  

  --lowestJoint = 

  print(unpack(j_heights))
  local mini={}
  print(util.min(j_heights))
  mini = util.min(j_heights)
  print(mini)
  --hcm.set_motion_headangle({0, 30*math.pi/180})

    t_start=t

end -- walk.update

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
  hcm.set_motion_headangle({0,0*math.pi/180})
end

return state

