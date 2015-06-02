local state = {}
state._NAME = ...
require'mcm'
-- IDLE IS A RELAX STATE
-- NOTHING SHOULD BE TORQUED ON HERE!

local Body = require'Body'
local timeout = 10.0
local t_entry, t_update

-- Running estimate of where the legs are
local qLLeg, qRLeg, qWaist

function state.entry()
  print(state._NAME..' Entry' ) 
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Torque OFF the motors
  Body.set_waist_torque_enable(1)
  Body.set_lleg_torque_enable({0,1,1, 0,0,0})
  Body.set_rleg_torque_enable({1,1,1, 0,0,0})

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


  local qLLeg = Body.get_lleg_position()
  local qRLeg = Body.get_rleg_position()

--  qLLeg[1],qLLeg[2],qLLeg[3] = 0, 3*DEG_TO_RAD, 62*DEG_TO_RAD
  qLLeg[1],qLLeg[2],qLLeg[3] = 0, 3*DEG_TO_RAD, 70*DEG_TO_RAD
  qRLeg[1],qRLeg[2],qRLeg[3] = 0, -5*DEG_TO_RAD, 63*DEG_TO_RAD


  Body.set_lleg_command_position(qLLeg)
  Body.set_rleg_command_position(qRLeg)

end

function state.exit()
  print(state._NAME..' Exit' ) 
end

return state
