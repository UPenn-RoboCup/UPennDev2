local state = {}
state._NAME = ...
local vector = require'vector'

local Body = require'Body'
local t_entry, t_update, t_finish

require'mcm'

require'dcm'

local qLArm, qRArm


local larm_pos_old,rarm_pos_old
local l_comp_torque,r_comp_torque

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

--[[
  Body.set_larm_torque_enable({2,2,2, 2,1,1,1}) --enable force control
  Body.set_rarm_torque_enable({2,2,2, 2,1,1,1}) --enable force control
--]]
  larm_pos_old = Body.get_larm_position()
  rarm_pos_old = Body.get_rarm_position()
  l_comp_torque = vector.zeros(7)
  r_comp_torque = vector.zeros(7)
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

--[[
  larm_pos = Body.get_larm_position()

  larm_torque = Body.get_larm_current()
  rarm_torque = Body.get_rarm_current()


print("larm pos:",util.print_jangle(larm_pos))
print(string.format("larm torque  :%.2f %.2f %.2f //   %.2f // %.2f %.2f // ",
  larm_torque[1],larm_torque[2],larm_torque[3],larm_torque[4],larm_torque[5],larm_torque[6]

  ))
print(string.format("rarm torque  :%.2f %.2f %.2f //   %.2f // %.2f %.2f // ",
  rarm_torque[1],rarm_torque[2],rarm_torque[3],rarm_torque[4],rarm_torque[5],rarm_torque[6]
  ))


   

   print(string.format("larm torque2 :%.2f %.2f %.2f //   %.2f // %.2f  // ",
  larm_torque2[1],larm_torque2[2],larm_torque2[3],larm_torque2[4],larm_torque2[5]
  ))
--]]
  

  local larm_cmdpos = Body.get_larm_command_position()
  local rarm_cmdpos = Body.get_rarm_command_position()
  local larm_pos = Body.get_larm_position()
  local rarm_pos = Body.get_rarm_position()
  l_stall_torque = vector.new(Body.Kinematics.calculate_arm_torque(larm_cmdpos))
  r_stall_torque = vector.new(Body.Kinematics.calculate_arm_torque(rarm_cmdpos))

--  l_stall_torque = Body.Kinematics.calculate_arm_torque(larm_pos)
--  r_stall_torque = Body.Kinematics.calculate_arm_torque(rarm_pos)



  local larm_vel =  (larm_pos-larm_pos_old)/dt
  local rarm_vel =  (rarm_pos-rarm_pos_old)/dt
  larm_pos_old,rarm_pos_old = larm_pos,rarm_pos

  local larm_pos_err = (larm_cmdpos-larm_pos)
  local rarm_pos_err = (rarm_cmdpos-rarm_pos)

  local l_comp_torque = util.pid_feedback(larm_pos_err, larm_vel, dt)
  local r_comp_torque = util.pid_feedback(rarm_pos_err, rarm_vel, dt)

  local l_torque =  l_stall_torque + l_comp_torque
  local r_torque =  r_stall_torque + r_comp_torque


--[[
  print(string.format("calculated left torque :%.2f %.2f %.2f //   %.2f // %.2f %.2f %.2f  // ",
  unpack(l_stall_torque)
  ))
  print(string.format("empirical left torque :%.2f %.2f %.2f //   %.2f // %.2f %.2f %.2f // ",
  unpack(l_torque)
  ))

  print(string.format("calculated right torque :%.2f %.2f %.2f //   %.2f // %.2f %.2f %.2f// ",
  r_stall_torque[1],r_stall_torque[2],r_stall_torque[3],r_stall_torque[4],r_stall_torque[5],r_stall_torque[6],r_stall_torque[7]
  ))

  print(string.format("empirical right torque :%.2f %.2f %.2f //   %.2f // %.2f %.2f %.2f // ",
  r_torque[1],r_torque[2],r_torque[3],r_torque[4],r_torque[5],r_torque[6],r_torque[7]
  ))
--]]



  ---(larm_cmdpos-larm_pos) +{}



  
  Body.set_larm_command_torque(l_torque)
  Body.set_rarm_command_torque(r_torque)



  --Body.set_larm_command_torque({0.1,0.1,  0,  -0.1,  0,0,0})



end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
