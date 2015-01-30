local state = {}
state._NAME = ...

--motionInit: initialize legs to correct position


require'mcm'
require'hcm'
local Body       = require'Body'
local K          = Body.Kinematics
local util       = require'util'
local vector     = require'vector'
local timeout    = 20.0
local t_readings = 0.20
local t_settle   = 0.10
local t_entry, t_update, t_finish
local moveleg = require'moveleg'


local err_th = 1*DEG_TO_RAD
if IS_WEBOTS then err_th = 5*DEG_TO_RAD end
-- Set the desired waist
local qWaist_desired = Config.stance.qWaist

local pTorso, qLLeg, qRLeg

local stage = 1
local t_last_debug
local last_error = 999

local bodyHeight = 1.0


function state.entry()
  print(state._NAME..' Entry' )
  Body.enable_read'lleg'
  Body.enable_read'rleg'

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t_entry
  t_last_debug=t_entry
  
  local footY    = Config.walk.footY
  local supportX = Config.walk.supportX

  --Generate current 2D pose for feet and torso
  local uTorso = vector.new({supportX, 0, 0})
  local uLeft  = util.pose_global(vector.new({-supportX, footY, 0}),uTorso)
  local uRight = util.pose_global(vector.new({-supportX, -footY, 0}),uTorso)
  mcm.set_status_uLeft(uLeft)
  mcm.set_status_uRight(uRight)
  mcm.set_status_uTorso(uTorso)
  mcm.set_status_uTorsoVel(vector.new{0,0,0})

  mcm.set_status_zLeg({0,0})

  mcm.set_stance_bodyHeight(Config.walk.bodyHeight)
  mcm.set_stance_bodyHeightTarget(Config.walk.bodyHeight)
  mcm.set_stance_bodyTilt(Config.walk.bodyTilt)
  hcm.set_motion_bodyHeightTarget(Config.walk.bodyHeight)

  mcm.set_status_uTorsoZMPComp({0,0,0})
  mcm.set_stance_uTorsoComp({0,0})
  mcm.set_status_iskneeling(0)

  mcm.set_walk_zShift({0,0})
  mcm.set_walk_zvShift({0,0})
  mcm.set_walk_aShiftX({0,0})
  mcm.set_walk_avShiftX({0,0})
  mcm.set_walk_aShiftY({0,0})
  mcm.set_walk_avShiftY({0,0})

  stage = 1
  if not IS_WEBOTS then
    print('INIT setting params')
    for i=1,10 do
      Body.set_head_command_velocity({500,500})
      unix.usleep(1e6*0.01);
      Body.set_waist_command_velocity({500,500})
      unix.usleep(1e6*0.01);
      Body.set_lleg_command_velocity({500,500,500,500,500,500})
      unix.usleep(1e6*0.01);
      Body.set_rleg_command_velocity({500,500,500,500,500,500})
      unix.usleep(1e6*0.01);
      Body.set_rleg_command_acceleration({50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
      Body.set_lleg_command_acceleration({50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
    end
  end

  mcm.set_motion_state(1.03)  
  hcm.set_legdebug_enable_balance({0,0})

  Body.set_waist_command_position({0,0})
end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()

  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  if Config.torque_legs then
    if IS_WEBOTS then
      bodyHeight = util.approachTol( bodyHeight, Config.walk.bodyHeight, Config.stance.dHeight, dt )
      mcm.set_stance_bodyHeight(bodyHeight)
    end

    delta_legs, angleShift = moveleg.get_leg_compensation_new(2,0,{0,0,0}, {0,0,0,0},dt)
    moveleg.set_leg_positions(delta_legs)

    local legBias = vector.new(mcm.get_leg_bias())
    local legBiasL = vector.slice(legBias,1,6)
    local legBiasR = vector.slice(legBias,7,12)

    local qLLegActual = Body.get_lleg_position() - legBiasL
    local qRLegActual = Body.get_rleg_position() - legBiasR
    local qWaistActual = Body.get_waist_position()

    local qLLegCommand = Body.get_lleg_command_position()
    local qRLegCommand = Body.get_rleg_command_position()
    local qWaistCommand = Body.get_waist_command_position()

    local err = 0
    for i=1,4 do --hack: skip ankle angles for now
      err = err + math.abs(qLLegActual[i]- qLLegCommand[i])
      err = err + math.abs(qRLegActual[i]- qRLegCommand[i])
    end
    err = err + math.abs(qWaistActual[1]- qWaistCommand[1])
    err = err + math.abs(qWaistActual[2]- qWaistCommand[2])

    if t>t_last_debug+1 then
      t_last_debug=t
      --SJ: we do have some steady steady error due to faulty servo (maybe)
      --so if the totall error does not decrase, we just exit
      if math.abs(last_error-err)<0.2*math.pi/180 then 
        print("Total joint reading err:",err*180/math.pi)
        return 'done'  
      end
      last_error = err
    end
  end
  

end

function state.exit()
  print(state._NAME..' Exit.  Time elapsed:',t_finish-t_entry )

  -- now on feet
  mcm.set_walk_bipedal(1)

  -- Update current pose
  
  

  local pg = Config.walk.leg_p_gain or 64
  local ag = Config.walk.ankle_p_gain or 64

  if not IS_WEBOTS then
    for i=1,10 do
      Body.set_head_command_velocity({6000,6000})
      unix.usleep(1e6*0.01);

      Body.set_waist_command_velocity({0,0})
      unix.usleep(1e6*0.01);


      if Config.torque_legs then

        Body.set_lleg_command_velocity({0,0,0,0,0,0})
        unix.usleep(1e6*0.01);

        Body.set_rleg_command_velocity({0,0,0,0,0,0})
        unix.usleep(1e6*0.01);

        Body.set_rleg_command_acceleration({0,0,0,0,0,0})
        unix.usleep(1e6*0.01);

        Body.set_lleg_command_acceleration({0,0,0,0,0,0})
        unix.usleep(1e6*0.01);

  --SJ: this somehow locks down head movement!!!!!!!!
        Body.set_head_position_p({pg,pg})
        unix.usleep(1e6*0.01);

        Body.set_rleg_position_p({pg,pg,pg,pg,pg,ag})
        unix.usleep(1e6*0.01);

        Body.set_lleg_position_p({pg,pg,pg,pg,pg,ag})
        unix.usleep(1e6*0.01);
      end
    end
  end
  mcm.set_walk_ismoving(0) --We are stopped

  --now disable leg joint reading
  --Body.disable_read'lleg'
  --Body.disable_read'rleg'
  wcm.set_robot_initdone(1)
end

return state
