--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------


--Armdrive: rotate left wrist roll joint to target angle

local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 15.0

-- Goal position is arm Init, with hands in front, ready to manipulate

local qLArm0,qRArm0

local qRArmCurrent, qRArmTarget, qLArmTarget, qLArmCurrent
local wheel_angle = 0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  --print("Dance Entry")
  stage = 1

  qLArm0 = Body.get_larm_command_position()
  qRArm0 = Body.get_rarm_command_position()

--[[
  qArmCurrent =  Body.get_larm_command_position()
  qArmTarget =  Body.get_larm_command_position()
--]]

  qRArmCurrent =  Body.get_rarm_command_position()
  qRArmTarget  =  Body.get_rarm_command_position()

  qLArmCurrent =  Body.get_larm_command_position()
  qLArmTarget  =  Body.get_larm_command_position()
  print(qLArmTarget)

  local pg = 4
  local tg = 16
  for i=1,10 do
    Body.set_rarm_position_p({pg,pg,pg,pg,pg,pg,tg})
    unix.usleep(1e6*0.01);

    Body.set_larm_position_p({pg,pg,pg,pg,pg,pg,tg})
    unix.usleep(1e6*0.01);
  end

end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  local i = 1
  local j = 1
  -- Save this at the last update time
  t_update = t
--  print(state._NAME..' Update' )

--  wheel_angle = wheel_angle + hcm.get_drive_wheel_angle()
  wheel_angle = hcm.get_drive_wheel_angle()
--[[
  local qRArmTarget = {
    qRArm0[1],qRArm0[2],qRArm0[3],
    qRArm0[4],qRArm0[5],qRArm0[6],
    qRArm0[7],
  }
   local qLArmTarget = {
    qLArm0[1],qLArm0[2],qLArm0[3],
    qLArm0[4],qLArm0[5],qLArm0[6],
    qLArm0[7],
  }
  --]]
  qRArmCurrent =  Body.get_rarm_command_position()
  qLArmCurrent =  Body.get_larm_command_position()

  --for i=1,20 do
    for j=1,7 do
      qRArmTarget[j] = -5*180*3.14+qRArmCurrent[j]
      qLArmTarget[j] = 5*180*3.14+qLArmCurrent[j]
    end

    --qRArmCurrent = util.approachTol(qRArmCurrent, qRArmTarget,
    --vector.new({0,0,0,0,0,0,1})*45*math.pi/180,dt)
    --qLArmCurrent = util.approachTol(qLArmCurrent, qLArmTarget,
    --vector.new({0,0,0,0,0,0,1})*45*math.pi/180,dt)


  --Body.set_rarm_command_position(qRArmCurrent) 
  
    Body.set_rarm_command_position(qRArmTarget)
    Body.set_larm_command_position(qLArmTarget) 
    
    --print("i = ")
    --print(i)
    unix.usleep(10000);

  --end
--[[
  for i=1,20 do
    for j=1,7 do
      qRArmTarget[j] = i/180*3.14+qRArmCurrent[j]
      qLArmTarget[j] = i/180*3.14+qLArmCurrent[j]
    end 
  
    Body.set_rarm_command_position(qRArmTarget)
    Body.set_larm_command_position(qLArmTarget) 
    
    print("i = ")
    print(i)
    unix.usleep(1000000);

  end
  
  --unix.usleep(10000);
  --]]


end

function state.exit()
  
  print(state._NAME..' Exit' )
end

return state
