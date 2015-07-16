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






--Gcm variables
-- 0 for initial / 1 for ready 2 for set / 3 for play / 4 fin
-- 5: Pre-initialized (idle) 6 for testing


local role = gcm.get_game_role()
local state = gcm.get_game_state()



	 if  key_char_lower==("1") then			
	 	if state~=0 then 
	 	  game_ch:send'init'
	 	end

	elseif key_char_lower==("2") then			
		if state~=1 then 
	 	  game_ch:send'ready'
	 	end
	elseif key_char_lower==("3") then			
		if state~=2 then 
		  game_ch:send'set'
		end
	elseif key_char_lower==("4") then  
		if state~=3 then 
		  game_ch:send'play'
		end
	elseif key_char_lower==("5") then  
		if state~=4 then 
		  game_ch:send'finish'		
		end

	elseif key_char_lower==("=") then  
		--Only change role at gameInitial or gameTest
		if (state==0 or state==6) and role~=1 then
			gcm.set_game_role(1)
		end

	elseif key_char_lower==("-") then  
		--Only change role at gameInitial or gameTest
		if (state==0 or state==6) and role~=0 then
			gcm.set_game_role(0)
		end

	elseif key_char_lower==("0") then  
		gcm.set_game_role(2)

	elseif key_char_lower==("9") then  
		--Demo mode on!
		gcm.set_game_role(3)
		

	elseif key_char_lower==("s") then  
		head_ch:send'scanobs'		
	elseif key_char_lower==("l") then  
		head_ch:send'log'		

	elseif key_char_lower==("8") then  
		mcm.set_walk_stoprequest(1)
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
