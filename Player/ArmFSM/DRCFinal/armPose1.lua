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

  Body.set_lleg_torque_enable({1,1,1, 1,1,2}) --enable force control
  Body.set_rleg_torque_enable({1,1,1, 1,1,2}) --enable force control

  larm_pos_old = Body.get_larm_position()
  rarm_pos_old = Body.get_rarm_position()
  l_comp_torque = vector.zeros(7)
  r_comp_torque = vector.zeros(7)
end

local count=0

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end


  count=count+1
  if count%100==0 then
    local lleg_cmdpos = Body.get_lleg_command_position()
    local rleg_cmdpos = Body.get_rleg_command_position()
    local lleg_pos = Body.get_lleg_position()
    local rleg_pos = Body.get_rleg_position()


    local lleg_actual_torque = Body.get_lleg_current()
    local rleg_actual_torque = Body.get_rleg_current()
    local lft = mcm.get_status_LFT()
    local rft = mcm.get_status_RFT()


    local qWaist = Body.get_waist_command_position()
    local qLArm = Body.get_larm_command_position()
    local qRArm = Body.get_rarm_command_position()

    local com_body_leftsupport = 
      --Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_cmdpos,rleg_cmdpos,0,0,0,  0,1)
      Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_pos,rleg_pos,0,0,0,  0,1)
    local com_body_rightsupport = 
      Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_cmdpos,rleg_cmdpos,0,0,0,  1,0)


    print("rel com w/o left leg:",
      com_body_leftsupport[1]/com_body_leftsupport[4],
      com_body_leftsupport[2]/com_body_leftsupport[4],
      com_body_leftsupport[3]/com_body_leftsupport[4]
      )
    print("total mass w/o left leg:",com_body_leftsupport[4] )

    lleg_stall_torque = vector.new(Body.Kinematics.calculate_leg_torque(lleg_cmdpos,1,com_body_leftsupport))



    rleg_stall_torque = vector.new(Body.Kinematics.calculate_leg_torque(rleg_cmdpos,0,com_body_rightsupport))

    print(string.format("LLeg actual torque: %.2f %.2f/  %.2f %.2f %.2f / %.2f",
        unpack(lleg_actual_torque)))
    print(string.format("RLeg actual torque: %.2f %.2f/  %.2f %.2f %.2f / %.2f",
        unpack(rleg_actual_torque)))


    print(string.format("LLeg calced torque: %.2f %.2f/  %.2f %.2f %.2f / %.2f",
        unpack(lleg_stall_torque)))

--[[
    print(string.format("RLeg actual torque: %.2f %.2f/  %.2f %.2f %.2f / %.2f",
        unpack(rleg_actual_torque)))
    print(string.format("RLeg calced torque: %.2f %.2f/  %.2f %.2f %.2f / %.2f",
        unpack(rleg_stall_torque)))
--]]
    print()
  end



--[[
----------------------------------------------------------------------------
-- Arm force-control code #1

  local larm_cmdpos = Body.get_larm_command_position()
  local rarm_cmdpos = Body.get_rarm_command_position()

  local larm_pos = Body.get_larm_position()
  local rarm_pos = Body.get_rarm_position()

--  l_stall_torque = Body.Kinematics.calculate_arm_torque(larm_pos)
--  r_stall_torque = Body.Kinematics.calculate_arm_torque(rarm_pos)

  l_stall_torque = vector.new(Body.Kinematics.calculate_arm_torque(larm_cmdpos))
  r_stall_torque = vector.new(Body.Kinematics.calculate_arm_torque(rarm_cmdpos))

  local larm_vel =  (larm_pos-larm_pos_old)/dt
  local rarm_vel =  (rarm_pos-rarm_pos_old)/dt
  larm_pos_old,rarm_pos_old = larm_pos,rarm_pos

  local larm_pos_err = (larm_cmdpos-larm_pos)
  local rarm_pos_err = (rarm_cmdpos-rarm_pos)

  local l_comp_torque = util.pid_feedback(larm_pos_err, larm_vel, dt)
  local r_comp_torque = util.pid_feedback(rarm_pos_err, rarm_vel, dt)

  local l_torque =  l_stall_torque + l_comp_torque
  local r_torque =  r_stall_torque + r_comp_torque

  Body.set_larm_command_torque(l_torque)
  Body.set_rarm_command_torque(r_torque)

----------------------------------------------------------------------------
--]]

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
