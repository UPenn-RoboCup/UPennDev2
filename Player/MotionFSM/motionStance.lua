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

-- Save gyro stabilization variables between update cycles
-- They are filtered.  TODO: Use dt in the filters
local angleShift = vector.new{0,0,0,0}

local uTorsoVel, velTorso
local tTorso0, tTorso1

local accTorso = Config.walk.accTorso or 0.5 --This works fine for now


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

  --Nonzero initial torso velocity handling
  uTorsoVel = mcm.get_status_uTorsoVel()
  velTorso = math.sqrt(uTorsoVel[1]*uTorsoVel[1]+
                          uTorsoVel[2]*uTorsoVel[2])
  tTorso0 = (1+1/math.sqrt(2)) * velTorso/accTorso
  tTorso1 = (1+math.sqrt(2)) * velTorso/accTorso

end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t


  local qWaist = Body.get_waist_command_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()    
--  local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm)

  --Quasi-static balancing


  local uTorso = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

--We disable this for now
--[[
-- Soft stopping of torso
-- Bang-bang strategy
-- x'' = -a for 0<t<t0,  x'' = a for t0<t<t1
-- x(0)=0, x'(0) = v0 , x(t1) = 0, x'(t1) = 0
-- x(t) = v0*t - a/2 * t^2 for 0<t<t0
-- x(t) = -a/2 * (t1-t)^2 for t0<t<t1
-- t0 = (1+1/sqrt(2)) (v0/a)
-- t1 = (1+ sqrt(2)) (v0/a)

  local torsoMag
  local t_passed = t-t_entry;
  if t_passed<tTorso0 then
    torsoMag = velTorso*t_passed - 0.5*accTorso*t_passed*t_passed
  elseif t_passed<tTorso1 then
    torsoMag = 0.5*accTorso*(tTorso1-t_passed)*(tTorso1-t_passed)
  else
    torsoMag = 0
  end
  if torsoMag>0 then
--    print(torsoMag)
  end
  if velTorso>0 then
    uTorso[1] = uTorso[1] + torsoMag * uTorsoVel[1]/velTorso
    uTorso[2] = uTorso[2] + torsoMag * uTorsoVel[2]/velTorso
  end
--]]

  mcm.set_walk_ismoving(0) --We stopped moving

  --Adjust body height
  local bodyHeight_now = mcm.get_stance_bodyHeight()  
  local bodyHeight = util.approachTol( bodyHeight_now, 
    Config.walk.bodyHeight, Config.stance.dHeight, t_diff )
  
  local zLeft,zRight = 0,0
  supportLeg = 2; --Double support
--[[ 
  -- ask for the foot sensor values
  Body.request_lfoot()
  Body.request_rfoot()
  --]]

-- Grab gyro feedback for these joint angles
  local gyro_rpy = Body.get_sensor_gyro()
  local delta_legs
 -- delta_legs, angleShift = moveleg.get_leg_compensation(supportLeg,0,gyro_rpy, angleShift)
  delta_legs, angleShift = moveleg.get_leg_compensation_simple(supportLeg,0,gyro_rpy, angleShift)

--print(angleShift[1],angleShift[2])

--Compensation for arm / objects
  local uTorsoComp = mcm.get_stance_uTorsoComp()


--[[
  local qWaist = Body.get_waist_command_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local massL = 5
  local massR = 0

  local com = K.com_upperbody(qWaist,qLArm,qRArm,
        Config.walk.bodyTilt, massL, massR)
  print("Com pos:", com[1]/com[4], com[2]/com[4])
  print("Torso compensation:",unpack(uTorsoComp))
--]]
 

  local uTorsoCompensated = util.pose_global({uTorsoComp[1],uTorsoComp[2],0},uTorso)


--  moveleg.set_leg_positions(uTorso,uLeft,uRight,
  mcm.set_stance_bodyHeight(bodyHeight)  
  moveleg.set_leg_positions(uTorsoCompensated,uLeft,uRight,  
    0,0,delta_legs)

  mcm.set_status_uTorsoVel({0,0,0})
end -- walk.update

function state.exit()



  mcm.set_stance_bodyHeight(Config.walk.bodyHeight)
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
end

return state
