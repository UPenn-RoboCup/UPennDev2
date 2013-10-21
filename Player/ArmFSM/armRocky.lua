--------------------------------
-- Humanoid arm state
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local state = {}
state._NAME = ...

local Body   = require'Body'
local util   = require'util'
local vector = require'vector'
local movearm = require'movearm'
local t_entry, t_update, t_finish
local timeout = 15.0

-- Goal position is arm Init, with hands in front, ready to manipulate

local qLArmTarget = 
 vector.new({90, 0, 0, -160, -90, -20,0})*Body.DEG_TO_RAD-- arms in front
local qRArmTarget =
 vector.new({90, 0, 0, -160, 90, 20,-0})*Body.DEG_TO_RAD-- arms in front


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  stage = 1
  Body.set_lgrip_percent(.7)
  Body.set_rgrip_percent(.7)

   if not IS_WEBOTS then
    for i=1,10 do
      Body.set_larm_command_velocity({500,500,500,500,500,500,500})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_velocity({500,500,500,500,500,500,500})
      unix.usleep(1e6*0.01);
      Body.set_larm_command_acceleration({50,50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_acceleration({50,50,50,50,50,50,50})
      unix.usleep(1e6*0.01);
    end
  end
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  movearm.setArmJoints(
    qLArmTarget,
    qRArmTarget,
    dt
    )

end

function state.exit()
  print(state._NAME..' Exit' )
  if not IS_WEBOTS then
    for i=1,10 do
      Body.set_larm_command_velocity({1700,1700,1700,1700,1700,1700,1700})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_velocity({1700,1700,1700,1700,1700,1700,1700})
      unix.usleep(1e6*0.01);
      Body.set_larm_command_acceleration({200,200,200,200,200,200,200})
      unix.usleep(1e6*0.01);
      Body.set_rarm_command_acceleration({200,200,200,200,200,200,200})
      unix.usleep(1e6*0.01);
    end
  end
end

return state
