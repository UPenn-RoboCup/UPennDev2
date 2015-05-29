#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'../fiddle.lua' end

estop = require'estop'

targetvel={0,0,0}
targetvel_new={0,0,0}
button_pressed = false

count=1
count2=1

function update_display()
--TODO: filtered zmp Y is inverted
  local LZMPr = dcm.get_sensor_lzmp()
  local RZMPr = dcm.get_sensor_rzmp()

  local zmpstr1 = string.format("ZL%d/%d R%d/%d",
			LZMPr[1]*1000, LZMPr[2]*1000,
			RZMPr[1]*1000, RZMPr[2]*1000)

  local velstr = string.format("Vel:%d %d cm %.1f",
		targetvel[1]*100, targetvel[2]*100, targetvel[3]);
--  count=count%4+1
  count=count+1


  if count%4==0 then
    estop.display(1,zmpstr1)

  elseif count%4==2 then
    estop.display(2,velstr)

  elseif count%10==5 then
	count2=count2+1

    estop.display(3,string.format("count %d",count2))

  elseif count%10==7 then
--	count2=count2+1
--    estop.display(4,string.format("count %d",count2))


  end


end



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
				motion_ch:send'hybridwalk'
			elseif ret.lstick[1]>600 then
				targetvel_new={0,0,0}
			elseif
				ret.lstick[1]<-600 then
				targetvel_new={0,0,0}
				body_ch:send'stop'
				button_pressed = true
				print("STOP")

			elseif ret.rbutton==1 then
				print("INIT!!!!!!")
				body_ch:send'init'	
	  		button_pressed = true		

			elseif ret.lstick[2]>600 then
				targetvel_new[3]=targetvel[3]+0.1
		  		button_pressed = true				
			elseif ret.lstick[2]<-600 then
				targetvel_new[3]=targetvel[3]-0.1
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
			end --end button check


		 	local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
		  if vel_diff>0 then
		    targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
		    print(string.format("Target velocity: %.3f %.3f %.3f",unpack(targetvel)))


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
	update_display()
	unix.usleep(1e6*0.1)
	
end

estop.close()
