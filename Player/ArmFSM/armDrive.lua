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

local qRArmCurrent, qRArmTarget


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  stage = 1

  qLArm0 = Body.get_larm_command_position()
  qRArm0 = Body.get_rarm_command_position()

  qLArmCurrent =  Body.get_larm_command_position()
  qLArmTarget =  Body.get_larm_command_position()

  local pg = 4
  local tg = 16
  for i=1,10 do
    Body.set_rarm_position_p({pg,pg,pg,pg,pg,pg,pg})
    unix.usleep(1e6*0.01);

    Body.set_larm_position_p({pg,pg,pg,pg,pg,pg,tg})
    unix.usleep(1e6*0.01);
  end

end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
--  print(state._NAME..' Update' )

  wheel_angle = hcm.get_drive_wheel_angle()

  local qLArmTarget = {
    qLArm0[1],qLArm0[2],qLArm0[3],
    qLArm0[4],qLArm0[5],qLArm0[6],
    qLArm0[7]+wheel_angle,
  }

  qLArmCurrent = util.approachTolRad(qLArmCurrent, qLArmTarget,
    vector.new({0,0,0,0,0,0,1})*45*math.pi/180,dt)

--print(qLArmTarget[7],qLArmCurrent[7])

  Body.set_larm_command_position(qLArmCurrent)  
end

function state.exit()
  
  print(state._NAME..' Exit' )
end

return state
