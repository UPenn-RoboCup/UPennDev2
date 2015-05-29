#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'../fiddle.lua' end

estop = require'estop'

targetvel={0,0,0}
targetvel_new={0,0,0}
button_pressed = false







function update_estop()
	ret = estop.update()


	if ret.estop==1 then
		print("ESTOP!!!!!!!!!!")
		--estop pressed, stop
		body_ch:send'stop'
	else --end estop

--  hack to stop the robot


		if not button_pressed then --don't do pogo stick

			if ret.lstick[1]>600 and ret.rstick[1]>600 then
				targetvel_new={0,0,0}
				button_pressed = true
				print("ZERO VEL")				
				motion_ch:send'hybridwalk'
			elseif
				ret.lstick[1]<-600 and ret.rstick[1]<-600 then
				targetvel_new={0,0,0}
				body_ch:send'stop'
				button_pressed = true
				print("STOP")

			elseif ret.rbutton==1 then
				print("INIT!!!!!!")
				body_ch:send'init'	
	  		button_pressed = true					
			elseif ret.lbutton[1]==1 then
				targetvel_new[1]=targetvel[1]+0.02;
	  		button_pressed = true				
			elseif ret.lbutton[2]==1 then
				targetvel_new[1]=targetvel[1]-0.02;
	  		button_pressed = true				
			elseif ret.lbutton[3]==1 then
				targetvel_new[2]=targetvel[2]+0.02;
	  		button_pressed = true				
			elseif ret.lbutton[4]==1 then
				targetvel_new[2]=targetvel[2]-0.02;
	  		button_pressed = true				
			elseif ret.rbutton==4 then
				targetvel_new[3]=targetvel[3]+0.1
	  		button_pressed = true				
			elseif ret.rbutton==2 then
				targetvel_new[3]=targetvel[3]-0.1
	  		button_pressed = true				
			end --end button check


		 	local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
		  if vel_diff>0 then
		    targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
		    print(string.format("Target velocity: %.3f %.3f %.3f",unpack(targetvel)))

		    ostring = string.format("Vel: %.1d %.1d %.2f",
			targetvel[1]*100, targetvel[2]*100, targetvel[3]);
			estop.display(1,ostring)

		    mcm.set_walk_vel(targetvel)
		  end --end vel update

	  else --button was released

	  	lbutton_chk = ret.lbutton[1]+ret.lbutton[2]+ret.lbutton[3]+ret.lbutton[4]
	  	if lbutton_chk==0 and ret.rbutton==0 and
				math.abs(ret.lstick[1])<300 and 
				math.abs(ret.rstick[1])<300 	  		then	
  		
	  		button_pressed = false   	end
	  end --end button release
	end --end estop
end

estop.init()

while true do
	update_estop()
	unix.usleep(1e6*0.1)
end

estop.close()
