#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local targetvel = {0,0,0}
local targetvel_new = {0,0,0}
local WAS_REQUIRED

local t_last = Body.get_time()
local tDelay = 0.005*1E6



DEG_TO_RAD = math.pi/180
RAD_TO_DEG = 1/DEG_TO_RAD


local throttle = 0
local steering = 0

local function update(key_code)
  if type(key_code)~='number' or key_code==0 then return end

  if Body.get_time()-t_last<0.2 then return end
  t_last = Body.get_time()

  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)

  if key_char_lower==("1") then      
    body_ch:send'init'
  elseif key_char_lower==("2") then      
    body_ch:send'driveready'
  elseif key_char_lower==("3") then      
    body_ch:send'drive'

  elseif key_char_lower==("j") then      
    steering = steering - 45*math.pi/180
    print("Steering angle:",steering*180/math.pi)
    hcm.set_teleop_steering(steering)

  elseif key_char_lower==("k") then      
    steering = 0
    print("Steering angle:",steering*180/math.pi)
    hcm.set_teleop_steering(steering)

  elseif key_char_lower==("l") then      
    steering = steering + 45*math.pi/180
    print("Steering angle:",steering*180/math.pi)
    hcm.set_teleop_steering(steering)


  elseif key_char_lower==("w") then
    throttle = math.min(throttle + 10*math.pi/180,30*math.pi/180)
    print("Throttle:  ",throttle*RAD_TO_DEG)
    hcm.set_teleop_throttle(throttle)

  elseif key_char_lower==("s") then
    throttle = 0
    print("Throttle: ",throttle*RAD_TO_DEG)
    hcm.set_teleop_throttle(throttle)

  elseif key_char_lower==("x") then
    throttle = math.max(throttle - 10*math.pi/180,0*math.pi/180)
    print("Throttle:  ",throttle*RAD_TO_DEG)
    hcm.set_teleop_throttle(throttle)


  elseif key_char_lower==(" ") then
    print("2 sec burst")
    hcm.set_teleop_throttle_duration(2)



  end
end



if ... and type(...)=='string' then
  WAS_REQUIRED = true
  return {entry=nil, update=update, exit=nil}
end

local getch = require'getch'
local running = true
local key_code
while running do
  key_code = getch.block()
  update(key_code)
end
