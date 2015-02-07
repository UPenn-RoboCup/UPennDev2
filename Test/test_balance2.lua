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

local function update(key_code)
	if type(key_code)~='number' or key_code==0 then return end
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
	--if gcm.get_game_state()==6 then --Testing state

	local lleft = hcm.get_legdebug_left()
	local lright = hcm.get_legdebug_right()
	local ltorso = hcm.get_legdebug_torso()

	local torsoangle = hcm.get_legdebug_torso_angle()


	if key_char_lower==("i") then      lright[1]=lright[1]+0.01
	elseif key_char_lower==("j") then  lright[2]=lright[2]+0.01
	elseif key_char_lower==("k") then  
		lright[4]=0
	elseif key_char_lower==("l") then  lright[2]=lright[2]-0.01
	elseif key_char_lower==(",") then  lright[1]=lright[1]-0.01
	elseif key_char_lower==("u") then  lright[4]=lright[4]+0.01
	elseif key_char_lower==("m") then  lright[4]=lright[4]-0.01

	elseif key_char_lower==("w") then  lleft[1]=lleft[1]+0.01
	elseif key_char_lower==("a") then  lleft[2]=lleft[2]+0.01
	elseif key_char_lower==("s") then  
		lleft[4]=0
	elseif key_char_lower==("d") then  lleft[2]=lleft[2]-0.01
	elseif key_char_lower==("x") then  lleft[1]=lleft[1]-0.01
	elseif key_char_lower==("q") then  lleft[4]=lleft[4]+0.01
	elseif key_char_lower==("z") then  lleft[4]=lleft[4]-0.01


	elseif key_char_lower==("t") then  ltorso[1]=ltorso[1]+0.01
	elseif key_char_lower==("f") then  ltorso[2]=ltorso[2]+0.01		
	elseif key_char_lower==("h") then  ltorso[2]=ltorso[2]-0.01
	elseif key_char_lower==("b") then  ltorso[1]=ltorso[1]-0.01
	elseif key_char_lower==("g") then  

	--[[		
	elseif key_char_lower==("-") then  torsoangle[1]=torsoangle[1]-math.pi/180		
	elseif key_char_lower==("=") then  torsoangle[1]=torsoangle[1]+math.pi/180			
	elseif key_char_lower==("[") then  torsoangle[2]=torsoangle[2]-math.pi/180		
	elseif key_char_lower==("]") then  torsoangle[2]=torsoangle[2]+math.pi/180			
	--]]					



	elseif key_char_lower==("1") then			
		body_ch:send'init'

	elseif key_char_lower==("2") then
		print("left lower")		
		hcm.set_legdebug_enable_balance({1,0})
						
	elseif key_char_lower==("3") then
		print("right lower")		
		hcm.set_legdebug_enable_balance({0,1})


	elseif key_char_lower==("4") then
		print("balance off")
		hcm.set_legdebug_enable_balance({0,0})


	elseif key_char_lower==("5") then      
		hcm.set_step_supportLeg(1)
		hcm.set_step_relpos({0.25,0,0})
		hcm.set_step_zpr({0.05,0,0})

		hcm.set_step_relpos({0.0,0,0})
		hcm.set_step_zpr({0.05,0,0})



		hcm.set_step_relpos({0.17,0,0})
		hcm.set_step_zpr({0.00,0,0})





		hcm.set_step_relpos({-0.28,0,0})
		hcm.set_step_zpr({-0.15,0,0})


		body_ch:send'stepover1'		

	elseif key_char_lower==("6") then      
--		hcm.set_step_relpos({0.50,0,0})
--		hcm.set_step_zpr({0.05,0,0})

		hcm.set_step_supportLeg(0)

		hcm.set_step_relpos({0.0,0,0})
		hcm.set_step_zpr({0.05,0,0})



		hcm.set_step_relpos({0.17,0,0})
		hcm.set_step_zpr({0.00,0,0})






		hcm.set_step_relpos({-0.28,0,0})
		hcm.set_step_zpr({-0.15,0,0})
		
		body_ch:send'stepover1'		


	elseif key_char_lower==("7") then      

		hcm.set_step_supportLeg(0)
		hcm.set_step_relpos({0.25,0,0})
		hcm.set_step_zpr({0.15,0,0})


		hcm.set_step_relpos({0.28,0,0})
		hcm.set_step_zpr({0.15,0,0})



		body_ch:send'stepover1'		

	elseif key_char_lower==("=") then      
		hcm.set_state_proceed(1)


	elseif key_char_lower==("8") then  
		hcm.set_step_supportLeg(1)
		hcm.set_step_relpos({0.25,0,0})
		hcm.set_step_zpr({0.15,0,0})


		hcm.set_step_relpos({0.28,0,0})
		hcm.set_step_zpr({0.15,0,0})

		body_ch:send'stepover1'		
	end

--[[
	elseif key_char_lower==("8") then  
		motion_ch:send'stand'
		body_ch:send'stop'
		if mcm.get_walk_ismoving()>0 then 
			print("requesting stop")
			mcm.set_walk_stoprequest(1) 
		end

	elseif key_char_lower==("9") then  
		motion_ch:send'hybridwalk'	
	end
--]]	
	hcm.set_legdebug_left(lleft)
	hcm.set_legdebug_right(lright)
	hcm.set_legdebug_torso(ltorso)
	hcm.set_legdebug_torso_angle(torsoangle)		

	print("foot hight:",lleft[4],lright[4])
	show_status()
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
