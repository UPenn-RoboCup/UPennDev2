#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local targetvel = {0,0,0}
local targetvel_new = {0,0,0}
local WAS_REQUIRED

local t_last = Body.get_time()
local tDelay = 0.005*1E6
local role_names={
  util.color('Goalie','green'),
  util.color('Attacker','red'),
  'Test'}
local gcm_names={
  util.color('Initial','green'),
  util.color('Ready','green'),
  util.color('Set','green'),
  util.color('Playing','blue'),
  util.color('Finished','green'),
  util.color('Untorqued','red'),
  util.color('Test','blue'),
}

local command1, command2 = '',''

local function show_status()

end


targetvel={0,0,0}
targetvel_new={0,0,0}


local function update(key_code)
	if type(key_code)~='number' or key_code==0 then return end
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
	--if gcm.get_game_state()==6 then --Testing state

	local lleft = hcm.get_legdebug_left()
	local lright = hcm.get_legdebug_right()
	local ltorso = hcm.get_legdebug_torso()


	local torsoangle = hcm.get_legdebug_torso_angle()
	if key_char_lower==("i") then    targetvel_new[1]=targetvel[1]+0.05;
  elseif key_char_lower==("j") then targetvel_new[3]=targetvel[3]+0.15;
  elseif key_char_lower==("k") then targetvel_new[1],targetvel_new[2],targetvel_new[3]=0,0,0;
  elseif key_char_lower==("l") then targetvel_new[3]=targetvel[3]-0.15;
  elseif key_char_lower==(",") then targetvel_new[1]=targetvel[1]-0.05;
  elseif key_char_lower==("h") then targetvel_new[2]=targetvel[2]+0.02;
  elseif key_char_lower==(";") then targetvel_new[2]=targetvel[2]-0.02;



	elseif key_char_lower=="1" then
		body_ch:send'init'

  elseif key_char_lower=="2" then
		head_ch:send'scan'



	elseif key_char_lower=="3" then
		head_ch:send'lookdown'

  elseif key_char_lower=="0" then
		head_ch:send'teleop'

  elseif key_char_lower=="g" then
    body_ch:send'approach'

	elseif key_char_lower==("5") then
		mcm.set_walk_kicktype(0)
		mcm.set_walk_kickfoot(0)--left foot kick
    mcm.set_walk_steprequest(1)
    mcm.set_walk_kickphase(1)
		
	elseif key_char_lower==("6") then
		mcm.set_walk_kicktype(0)
 		mcm.set_walk_kickfoot(1)--left foot kick
    mcm.set_walk_steprequest(1)
    mcm.set_walk_kickphase(1)
		
	elseif key_char_lower==("t") then
		mcm.set_walk_kicktype(1)
		mcm.set_walk_kickfoot(0)--left foot kick
    mcm.set_walk_steprequest(1)
    mcm.set_walk_kickphase(1)

	elseif key_char_lower==("y") then
		mcm.set_walk_kicktype(1)
 		mcm.set_walk_kickfoot(1)--left foot kick
    mcm.set_walk_steprequest(1)
    mcm.set_walk_kickphase(1)
		



--[[
	elseif key_char_lower==("3") then
		hcm.set_step_supportLeg(0)
		hcm.set_step_relpos({0.08,0,0})
		hcm.set_step_zpr({0.00,0,0})
		body_ch:send'stepflat'

	elseif key_char_lower==("4") then
		hcm.set_step_supportLeg(1)
		hcm.set_step_relpos({0.08,0,0})
		hcm.set_step_zpr({0.00,0,0})
		body_ch:send'stepflat'


elseif key_char_lower==("5") then
		hcm.set_step_supportLeg(0)
		hcm.set_step_relpos({0.16,0,0})
		hcm.set_step_zpr({0.00,0,0})
		body_ch:send'stepflat'

	elseif key_char_lower==("6") then
		hcm.set_step_supportLeg(1)
		hcm.set_step_relpos({0.16,0,0})
		hcm.set_step_zpr({0.00,0,0})
		body_ch:send'stepflat'




	elseif key_char_lower==("=") then
		hcm.set_state_proceed(1)
--]]


	elseif key_char_lower==("8") then
		body_ch:send'stop'
		mcm.set_walk_stoprequest(1)

	elseif key_char_lower==("9") then
		motion_ch:send'hybridwalk'
	end


 	local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
  if vel_diff>0 then
    targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
    print(string.format("Target velocity: %.3f %.3f %.3f",unpack(targetvel)))
    mcm.set_walk_vel(targetvel)
  end
end

show_status()
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
