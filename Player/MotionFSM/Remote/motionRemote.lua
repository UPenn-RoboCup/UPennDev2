local state = {}
state._NAME = ...

local RemoteControl = require'RemoteControl.ffi'

local timeout = 5.0
local t_entry, t_update
local rc

function state.entry()
  print(state._NAME..' Entry')

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  rc = rc or RemoteControl.init('192.168.123.77')
  print('Interface', rc, rc.n)
	-- Torque off most things
	Body.set_head_torque_enable(0)
	Body.set_waist_torque_enable(0)
	Body.set_larm_torque_enable(0)
	Body.set_rarm_torque_enable(0)
	--Body.set_rleg_torque_enable(0)
	--Body.set_lleg_torque_enable(0)
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
  if t - t_entry > timeout then return'timeout' end

  -- Process
  rc:send():wait()
  
  repeat
    rc:receive()
    rc:process()
  until not rc.cmds

end

function state.exit()
  print(state._NAME..' Exit' ) 
end

return state
