local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'

local handle_pos, handle_yaw, handle_pitch, handle_radius, turnAngle=0,0,0,0,0
local lShoulderYaw, rShoulderYaw = 0,0

local dqArmMax = Config.arm.fast_limit 
local dpArmMax = Config.arm.linear_slow_limit

local dturnAngleMax = 3*math.pi/180 -- 3 deg per sec
local dpHandleMax = {0.01, 0.01, 0.01}
local drHandleMax = 0.01
local daHandleMax = 3*math.pi/180 
local dturnAngleMax = 6*math.pi/180 -- 3 deg per sec


--local dturnAngleMax = 15*math.pi/180 -- 3 deg per sec


local turn_state = 1
local turnAngleTarget0 = 10*math.pi/180
--local turnAngleTarget1 = -55*math.pi/180
local turnAngleTarget1 = -35*math.pi/180

local gripL,gripR = 0,0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  local wheel   = hcm.get_wheel_model()    
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  turnAngle = 0
  hcm.set_wheel_turnangle(0)
  turn_state = 1
end

local t_debug = Body.get_time()

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  --Update handle model using interpolation
  local wheel   = hcm.get_wheel_model()
  local turnAngle1 = hcm.get_wheel_turnangle()
  local handle_pos1    = vector.slice(wheel,1,3)
  local handle_yaw1    = wheel[4]
  local handle_pitch1  = wheel[5]
  local handle_radius1 = wheel[6]

  local handle_pos2 = util.approachTol(handle_pos, handle_pos1, dpHandleMax,dt)
  local handle_yaw2 = util.approachTol(handle_yaw,handle_yaw1, daHandleMax,dt)
  local handle_pitch2 = util.approachTol(handle_pitch,handle_pitch1, daHandleMax,dt)
  local handle_radius2 = util.approachTol(handle_radius, handle_radius1,drHandleMax,dt)    
--  local turnAngle2,doneA = util.approachTol(turnAngle,turnAngle1,dturnAngleMax, dt )
  local turnAngle2,doneA
  local doneL,doneR
  if turn_state%4==1 then
    gripL,doneL = util.approachTol(gripL,0,2,dt)
    gripR,doneR = util.approachTol(gripR,1,2,dt)    
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
  elseif turn_state%4==2 then
    turnAngle2,doneA = util.approachTol(turnAngle1,turnAngleTarget0,dturnAngleMax, dt )
  elseif turn_state%4==3 then
    gripL,doneL = util.approachTol(gripL,1,2,dt)
    gripR,doneR = util.approachTol(gripR,0,2,dt)    
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
  elseif turn_state%4==0 then    
    turnAngle2,doneA = util.approachTol(turnAngle1,turnAngleTarget1,dturnAngleMax, dt )
  end
    
  if turn_state%2==0 then  
    --Adaptive shoulder yaw angle
    ret = movearm.setArmToWheelPosition(
      handle_pos2, handle_yaw2, handle_pitch2,
      handle_radius2, turnAngle2,dt)
      
    if ret==-1 or doneA then    
       hcm.set_wheel_model({handle_pos[1],handle_pos[2],handle_pos[3],
       handle_yaw,handle_pitch,handle_radius})
       hcm.set_wheel_turnangle(turnAngle)    
       turn_state = turn_state+1;
    else
      handle_pos,handle_yaw,handle_pitch,handle_radius,turnAngle=
        handle_pos2,handle_yaw2,handle_pitch2,handle_radius2,turnAngle2 
      hcm.set_wheel_turnangle(turnAngle2)    
    end
  else
    if doneL and doneR then turn_state = turn_state+1 end
  end


end

function state.exit()
  print(state._NAME..' Exit' )
end

return state