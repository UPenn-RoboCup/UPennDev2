local state = {}
state._NAME = ...
require'mcm'
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!

local Body = require'Body'
local timeout = 10.0
local t_entry, t_update

-- Running estimate of where the legs are
local qRLeg0
local throttle, throttle_old = 0,0


function state.entry()
  print(state._NAME..' Entry' ) 
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Torque OFF the motors
  Body.set_waist_torque_enable(1)
  Body.set_lleg_torque_enable(1)
  Body.set_rleg_torque_enable(1)

--
  if not IS_WEBOTS then
    for i=1,10 do
--      Body.set_lleg_command_velocity({500,500,500,500,500,500})
      Body.set_lleg_command_velocity({500,500,500,2000,2000,500})

      unix.usleep(1e6*0.01);
--      Body.set_rleg_command_velocity({500,500,500,500,500,500})
      Body.set_rleg_command_velocity({500,500,500,2000,2000,500})




      unix.usleep(1e6*0.01);
      Body.set_rleg_command_acceleration({50,50,50,200,200,50})
      unix.usleep(1e6*0.01);
      Body.set_lleg_command_acceleration({50,50,50,200,200,50})
      unix.usleep(1e6*0.01);
    end
  end
--
  qLLeg0 = Body.get_lleg_command_position()
  qRLeg0 = Body.get_rleg_command_position()
  throttle, throttle_old = 0,0

end

---
--Set actuator commands to resting position, as gotten from joint encoders.
function state.update()
--  print(state._NAME..' Update')
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t

  local throttleT = math.min(1,math.max(-1,hcm.get_teleop_throttle() ))

  throttleT = (throttleT-0.1)/0.9 --go back a bit 

  local throttle_pitch_mag = Config.throttle_pitch_mag or 15*math.pi/180
  
  throttle = util.approachTol(throttle, throttleT*throttle_pitch_mag, 1, t_diff)

  if Config.birdwalk then
    local qLLeg = util.shallow_copy(qLLeg0)
    local qLLegCommand = Body.get_lleg_command_position()

    qLLeg[4] = qLLeg[4] + throttle*0.3
    qLLeg[5] = qLLeg[5] - throttle

--[[
    local qLLeg_approach = util.approachTolRad( 
	qLLegCommand, qLLeg,
	{30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD, 30*DEG_TO_RAD, 30*DEG_TO_RAD}  
	,t_diff, ,nil,absolute)


    Body.set_lleg_command_position(qLLeg_approach)
--]]
    Body.set_lleg_command_position(qLLeg)

  else
    local qRLeg = util.shallow_copy(qRLeg0)
    qRLeg[5] = qRLeg[5]+throttle_pitch_mag * throttle
    Body.set_rleg_command_position(qRLeg)
  end

end

function state.exit()
  print(state._NAME..' Exit' ) 
  Body.set_lleg_torque_enable(1)
  Body.set_rleg_torque_enable(1)

end

return state
