local state = {}
state._NAME = ...
local vector = require'vector'

local Body = require'Body'
local t_entry, t_update, t_finish, tLastUpdate
local timeout = 10.0
require'mcm'

local qLArm, qRArm

local mode = 0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

  --Untorque both arms
  for i=1,10 do
    Body.set_larm_torque_enable(0)
    unix.usleep(1e6*0.01);
    Body.set_rarm_torque_enable(0)
    unix.usleep(1e6*0.01);
  end
  tLastUpdate = t_entry
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  --Read arm positions (for switching roles purpose)
  qLArm = Body.get_larm_position()
  qRArm = Body.get_rarm_position()  

  local threshold = 45*DEG_TO_RAD


  local qL = util.mod_angle(qLArm[1])
  local qR = util.mod_angle(qRArm[1])

  local newmode = mode
  if qL>threshold and qR>threshold then
    mode = 1
  elseif qL<-threshold and qR<-threshold then
    mode = 2
  else
    mode = 0
  end

  if t-tLastUpdate>0.2 then
    if mode==0 then
      Body.set_head_led_red(0)
      Body.set_head_led_green(128)
      Body.set_head_led_blue(0)
      if gcm.get_game_state()~=6 then
        game_ch:send'stop'
      end
    elseif mode==1 then
      Body.set_head_led_red(255)
      Body.set_head_led_green(0)
      Body.set_head_led_blue(0)
    elseif mode==2 then
      Body.set_head_led_red(0)
      Body.set_head_led_green(0)
      Body.set_head_led_blue(255)

    end
  tLastUpdate = t
  end

end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
