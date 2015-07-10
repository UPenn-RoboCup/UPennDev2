local state = {}
state._NAME = ...

local Body = require'Body'
local t_entry, t_update, t_exit

function state.entry()
  print(state._NAME..' Entry' )

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry

  -- Initialize all other state machines
  arm_ch:send'estop'
  motion_ch:send'estop'
  head_ch:send'estop'
  lidar_ch:send'estop'
  gripper_ch:send'close'
  hcm.set_teleop_estop(1)

  Body.set_larm_torque_enable(0)
  Body.set_rarm_torque_enable(0)
  Body.set_lleg_torque_enable(0)
  Body.set_rleg_torque_enable(0)
  Body.set_head_torque_enable(0)
end

function state.update()
  --  print(state._NAME..' Update' ) 
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  --TODO: Check whether all FSMs have done initialzing 
  local body_init = mcm.get_status_body_init()
  local arm_init = mcm.get_status_arm_init()

  --TODO
--  return 'done'
  local estop = hcm.get_teleop_estop()
  if estop==0 then
    print("ESTOP RELEASED!")
    return "done"
  end

end

function state.exit()
  --Release velocity limits for all the servos here
  print(state._NAME..' Exit' )
  t_exit = Body.get_time()
end

return state
