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

----------------------------------------------------------
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

---------------------------
-- State machine methods --
---------------------------
local t0
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  mcm.set_walk_bipedal(1)
  t0 = t_entry
  ----[[
  --Body.set_lleg_command_velocity({17000,17000,17000,17000,17000,17000})
  --Body.set_rleg_command_velocity({17000,17000,17000,17000,17000,17000})
  Body.set_rleg_command_acceleration({200,200,200,200,200,200})
  
  --]]
  --Body.set_lleg_command_velocity({500,500,500,500,500,500})
  --Body.set_rleg_command_velocity({500,500,500,500,500,500})
end
cnt = 0
function state.update()
  --Body.set_lleg_command_velocity({17000,17000,17000,17000,17000,17000})
  --Body.set_rleg_command_velocity({25000,25000,25000,25000,25000,25000})
  Body.set_rleg_command_acceleration({200,200,200,200,200,200})
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  
  local t_passed = t-t0

--Read stored feet and torso poses 
  local uTorso = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  --Adjust body height
  local bodyHeight_now = mcm.get_status_bodyHeight()  
  local bodyHeight = util.approachTol( bodyHeight_now, 
    Config.walk.bodyHeight, Config.stance.dHeight, t_diff )
  
  local zLeft,zRight = 0,0
  supportLeg = 2; --Double support

  local t_period = 0.50
  zLeft = 0.05* ( 0.5* math.sin(2*math.pi*t_passed / t_period) + 0.5)
  zRight = 0.05* (0.5* math.sin(2*math.pi*t_passed / t_period) + 0.5)

--[[
  local t_period = 2
  if t_passed%t_period<t_period/2 then
    zLeft = 0
    zRight = 0
  else
    zLeft = 0.05
    zRight = 0.05
  end
--]]



  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  -- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local delta_legs
  gyro_rpy = vector.zeros(3)
  delta_legs, ankleShift, kneeShift, hipShift = moveleg.get_leg_compensation(
      supportLeg,0,gyro_rpy, ankleShift, kneeShift, hipShift, 0)

  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight,
        0,bodyTilt,uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})

moveleg.set_leg_positions(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)

  qLeg = Body.get_lleg_command_position()
  print(string.format("%.3f %.2f %.2f %.2f",
   t_passed, qLeg[3]*Body.RAD_TO_DEG, qLeg[4]*Body.RAD_TO_DEG, qLeg[5]*Body.RAD_TO_DEG))

  mcm.set_status_bodyHeight(bodyHeight)  
end -- walk.update

function state.exit()
  mcm.set_status_bodyHeight(Config.walk.bodyHeight)
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state
