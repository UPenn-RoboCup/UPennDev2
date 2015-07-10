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
      Body.set_lleg_command_velocity({500,500,500,1000,3000,500})
      unix.usleep(1e6*0.01);
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

  t_throttle_end = 0
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

  local throttleT = hcm.get_teleop_throttle0() 
  local throttle_duration = hcm.get_teleop_throttle_duration()

  if throttle_duration>0 then --start pedalling
    t_throttle_end = t + throttle_duration
    hcm.set_teleop_throttle_duration(0)
  end

  if t_throttle_end>t then
    throttleT = throttleT + 
	math.min(30*math.pi/180,math.max(0,
      	hcm.get_teleop_throttle() ))
  end

  throttle = util.approachTol(throttle, throttleT, 90*math.pi/180, t_diff)

  --we are doing birdwalk and facing backwards
  --so we use left foot to press the pedal
  local qLLeg = util.shallow_copy(qLLeg0)
  local qLLegCommand = Body.get_lleg_command_position()

  qLLeg[4] = qLLeg[4] + throttle*0.3
  qLLeg[5] = qLLeg[5] - throttle
  Body.set_lleg_command_position(qLLeg)

end

function state.exit()
  print(state._NAME..' Exit' ) 
  Body.set_lleg_torque_enable(1)
  Body.set_rleg_torque_enable(1)

end

return state
