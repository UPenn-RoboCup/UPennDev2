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


local movement_target=vector.new({0,0,0})



local char_to_movetarget = {
	['w'] = vector.new({0.025, 0, 0}),
  ['x'] = vector.new({-.025, 0, 0}),
  ['a'] = vector.new({0, 0.025, 0}),
  ['d'] = vector.new({0, -.025, 0}),
  ['q'] = vector.new({0, 0, 5*math.pi/180}),
  ['e'] = vector.new({0,0, -5*math.pi/180}),
}


local function print_override()
  print( util.color('Override:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f %.1f",
      override_target[1],
      override_target[2],
      override_target[3],
      override_target[4]*180/math.pi,
      override_target[5]*180/math.pi,
      override_target[6]*180/math.pi
      ))
end

local stair_height = 0.225

local function update(key_code)
  if type(key_code)~='number' or key_code==0 then return end

  if Body.get_time()-t_last<0.2 then return end
  t_last = Body.get_time()

  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)


  if key_char_lower==("1") then      
    body_ch:send'init'
    arm_ch:send'init'  --initialize arm to walk position 


elseif key_char_lower==("t") then      
    hcm.set_step_supportLeg(0)    
    hcm.set_step_zpr({0,0,0})
    hcm.set_step_relpos({0.35,0.01,0})    
    hcm.set_step_zpr({stair_height,0,0}) --stair    
    body_ch:send'stairclimb'   

elseif key_char_lower==("y") then      
    hcm.set_step_supportLeg(1)
    hcm.set_step_zpr({0.00,0,0})
    hcm.set_step_relpos({0.35,-0.01,0})
    hcm.set_step_zpr({stair_height,0,0}) --stair    
    body_ch:send'stairclimb'   

elseif key_char_lower==("3") then    
    hcm.set_step_supportLeg(0)  
    hcm.set_step_relpos({0.30,0,0})
    hcm.set_step_zpr({stair_height,0,0}) --stair    
    body_ch:send'stairclimb'   

elseif key_char_lower==("4") then      
    hcm.set_step_supportLeg(1) --move rfoot
    hcm.set_step_relpos({0.30,0,0})
    hcm.set_step_zpr({stair_height,0,0}) --stair
    body_ch:send'stairclimb'   

elseif key_char_lower==("5") then 
    hcm.set_step_supportLeg(0) --move lfoot
    hcm.set_step_relpos({-0.30,0,0})
    hcm.set_step_zpr({-stair_height,0,0}) --stair
    body_ch:send'stairclimb' 

elseif key_char_lower==("6") then 
    hcm.set_step_supportLeg(1) --move rfoot
    hcm.set_step_relpos({-0.30,0,0})
    hcm.set_step_zpr({-stair_height,0,0}) --stair
    body_ch:send'stairclimb' 

	elseif key_char_lower==("8") then  
		body_ch:send'stop'
  elseif key_char_lower==(" ") then
    hcm.set_teleop_waypoint(movement_target)
		body_ch:send'approach' --todo
    movement_target = vector.zeros(3)
  end

 	local movetarget_adj = char_to_movetarget[key_char_lower]
 	if movetarget_adj then
 		movement_target=movement_target+movetarget_adj
 		print( util.color('Move target: ','yellow'), 
 			string.format("%.3f %.3f %.1f",movement_target[1],movement_target[2],movement_target[3]*180/math.pi )
 			)
 		return
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
