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
local bodyHeight_next
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
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  mcm.set_walk_bipedal(1)  
  bodyHeight_next = mcm.get_status_bodyHeight()  
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  
--Read stored feet and torso poses 
  local uTorso = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  --Adjust body height
  local bodyHeight_now = mcm.get_status_bodyHeight()  
  local bodyHeight_target = hcm.get_motion_bodyHeightTarget()



  bodyHeight_target = math.min(Config.walk.bodyHeight,
    math.max(Config.stance.sitHeight, bodyHeight_target))

bodyHeight_target = 0.70; --Hack

bodyHeight_target = 0.47; --Full kneel down

  bodyHeight_next = util.approachTol( bodyHeight_now, 
    bodyHeight_target, Config.stance.dHeight, t_diff )

  local zLeft,zRight = 0,0
  supportLeg = 2; --Double support

  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  -- Grab gyro feedback for these joint angles
  local gyro_rpy = moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local delta_legs
  delta_legs, ankleShift, kneeShift, hipShift = moveleg.get_leg_compensation(
      supportLeg,0,gyro_rpy, ankleShift, kneeShift, hipShift, 0)

  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight_next,
        0,bodyTilt,uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
    
  moveleg.set_leg_positions(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)  
  mcm.set_status_bodyHeight(bodyHeight_next)    
end -- walk.update

function state.exit()
  
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state