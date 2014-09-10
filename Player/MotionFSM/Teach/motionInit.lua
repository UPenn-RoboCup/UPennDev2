-- motionInit: initialize legs to correct position
local state = {}
state._NAME = ...

require'mcm'
require'hcm'
local Body = require'Body'
local K = Body.Kinematics
local moveleg = require'moveleg'
local util = require'util'
local vector = require'vector'
local timeout = 20.0
local t_entry, t_update

-- Set the desired leg and torso poses
local pLLeg_desired = vector.new{-Config.walk.supportX,  Config.walk.footY, 0, 0,0,0}
local pRLeg_desired = vector.new{-Config.walk.supportX,  -Config.walk.footY, 0, 0,0,0}
local pTorso_desired = vector.new{-Config.walk.torsoX, 0, Config.walk.bodyHeight, 0,Config.walk.bodyTilt,0}
local qWaist_desired = Config.stance.qWaist

-- Tolerance on the waist
local qWaist_tol = vector.ones(2) * DEG_TO_RAD
-- How far away to tell the P controller to go in one step
local dqWaistSz = vector.ones(2) * 2 * DEG_TO_RAD

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t_entry
  
  Body.enable_read'lleg'
  Body.enable_read'rleg'

  -- Hardware accomodation
  if not IS_WEBOTS then
    -- Set speed limits for initial moving
    for i=1,5 do
      Body.set_head_command_velocity({500,500})
      Body.set_waist_command_velocity({500,500})
      Body.set_lleg_command_velocity({500,500,500,500,500,500})
      Body.set_rleg_command_velocity({500,500,500,500,500,500})
      Body.set_rleg_command_acceleration({50,50,50,50,50,50})
      Body.set_lleg_command_acceleration({50,50,50,50,50,50})
      unix.usleep(1e4)
    end
  end
  
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()

  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if t-t_entry > timeout then return'timeout' end

  -- Zero the waist
  local qWaist = Body.get_waist_position()
  local qWaist_approach, doneWaist = util.goto(qWaist, qWaist_desired, dqWaistSz, qWaist_tol)
  Body.set_waist_command_position(qWaist_approach)

  if not Config.torque_legs then return doneWaist and 'done' end

  -- Move legs if they are torqued on
  local doneLegs = moveleg.set_lower_body_slowly(
    pTorso_desired, pLLeg_desired, pRLeg_desired
  )
  
  return doneLegs and doneWaist and 'done'

end

function state.exit()
  print(string.format('%s Exit | %.3f seconds elapsed', state._NAME, Body.get_time()-t_entry))
  
  -- Update current pose
  -- Generate current 2D pose for feet and torso
  local footY    = Config.walk.footY
  local supportX = Config.walk.supportX
  local uTorso = vector.new({supportX, 0, 0})
  local uLeft  = util.pose_global(vector.new({-supportX, footY, 0}), uTorso)
  local uRight = util.pose_global(vector.new({-supportX, -footY, 0}), uTorso)
  mcm.set_status_uLeft(uLeft)
  mcm.set_status_uRight(uRight)
  mcm.set_status_uTorso(uTorso)
  mcm.set_status_uTorsoVel({0,0,0})
  mcm.set_stance_uTorsoComp({0,0})
  mcm.set_status_iskneeling(0)
  mcm.set_walk_bipedal(1)
  mcm.set_walk_ismoving(0) -- We are stopped
  wcm.set_robot_initdone(1)

  -- Real hardware accomodation
  if not IS_WEBOTS then
    -- Gains
    local pg = Config.walk.leg_p_gain or 64
    local ag = Config.walk.ankle_p_gain or 64
    local unlimited = vector.zeros(6)
    local p_legs = {pg,pg,pg,pg,pg,ag}
    local p_head = {pg,pg}
    -- Update the limits
    -- Setting the command accel and command velocity to zero means no limit for vel and accel
    for i=1,5 do
      -- {2000,2000}
      Body.set_head_command_velocity({6000,6000})
      Body.set_waist_command_velocity({0,0})
      if Config.torque_legs then
        --{17000,17000,17000,17000,17000,17000}
        Body.set_lleg_command_velocity(unlimited)
        Body.set_rleg_command_velocity(unlimited)
        --{200,200,200,200,200,200}
        Body.set_lleg_command_acceleration(unlimited)
        Body.set_rleg_command_acceleration(unlimited)
        --SJ: this somehow locks down head movement!!!!!!!!
        Body.set_head_position_p(p_head)
        Body.set_rleg_position_p(p_legs)
        Body.set_lleg_position_p(p_legs)
      end
      unix.usleep(1e4)
    end
  end -- hardware accomodation
end --exit

return state
