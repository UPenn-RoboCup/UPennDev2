#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'../fiddle.lua' end

estop = require'estop'


display_enable={1,0,0,0}
display_start_t={0,0,0,0}
display_msg={"T1","T2","",""}
display_count =0
display_last_update_t = 0

display_update_duration = 2.0
display_update_interval = 0.2
command_update_interval = 0.1


function update_display_msg(line,text)
	local t = Body.get_time()
	display_enable[line]=1
	display_start_t[line]=t
	display_msg[line]=text
end

function refresh_display()		
	--only send one line every 0.25 sec, so do round robin 
	local t = Body.get_time()
	if t-display_last_update_t<display_update_interval then return end 

	--find next message to update
	if display_enable[1]
		+display_enable[2]
		+display_enable[3]
		+display_enable[4]==0 then return end
	local next_not_found = true
	while next_not_found do
		display_count = display_count%4 +1
		if display_enable[display_count]==1 then next_not_found=false end
	end
	estop.display(display_count, display_msg[display_count])

	--don't update old messages
	if t-display_start_t[display_count]>display_update_duration then
		display_enable[display_count]=0
	end
	display_last_update_t = t
end

local steer, gas = 0,0
local vel={0,0,0}
targetvel={0,0,0}
targetvel_new={0,0,0}



display_mode_old = 0
display_mode = 1 --Walk test mode
button_pressed = false

function update_display()
	local batt = mcm.get_status_battery()/10

	--persistent message
	if display_mode_old~=display_mode then
		display_mode_old = display_mode
		if display_mode==1 then
			update_display_msg(1,"4<<   Walk Test     ")
			update_display_msg(2, string.format("Vel: %3i %3i %3i",vel[1],vel[2],vel[3]) )
			update_display_msg(3, string.format("Battery: %.1f",batt) )
		elseif display_mode==2 then
			update_display_msg(1,"    Driving Test >>4")
			update_display_msg(3,"1-init  2-driveready")
			update_display_msg(4,"3-drive             ")
		end
	end

	--messages that gets updated every time
	if display_mode==1 then
		local LZMPr = dcm.get_sensor_lzmp()
    local RZMPr = dcm.get_sensor_rzmp()
    local zmpstr1 = string.format("ZL%3i/%3i R%3i/%3i",
			LZMPr[1]*1000, LZMPr[2]*1000,
			RZMPr[1]*1000, RZMPr[2]*1000)
		update_display_msg(3, zmpstr1 )
 		update_display_msg(4, string.format("Battery: %.1f",batt) )

	elseif display_mode==2 then
		update_display_msg(2,string.format("Steer: %3i Gas: %3i",steer*100, gas*100 ))
		update_display_msg(3, string.format("Battery: %.1f",batt) )

	end
end


local stick_center = 300

function process_click(ret)
	--Button press handling (no pogo stick)
	local lbutton_chk = (ret.lbutton[1]+ret.lbutton[2]+ret.lbutton[3]+ret.lbutton[4] == 0)
	local rbutton_chk = (ret.rbutton==0)
	local lstick_chk = 
		math.abs(ret.lstick[1])<stick_center and
		math.abs(ret.lstick[2])<stick_center and
		math.abs(ret.lstick[3])<stick_center

	local rstick_chk = 
		math.abs(ret.rstick[1])<stick_center and
		math.abs(ret.rstick[2])<stick_center and
		math.abs(ret.rstick[3])<stick_center

	if button_pressed then
		if lbutton_chk and rbutton_chk and lstick_chk and rstick_chk then button_pressed=false end
		return
	end
	button_pressed = true


	if display_mode==1 then
		if ret.rbutton==4 then 
			display_mode = 2
			return
		elseif ret.rbutton==1 then
			body_ch:send'init'	
		elseif ret.lstick[1]>600 then
			targetvel_new={0,0,0}
			motion_ch:send'hybridwalk'
		elseif ret.lstick[1]<-600 then
			targetvel_new={0,0,0}
			body_ch:send'stop'
		elseif ret.lbutton[1]==1 then
			targetvel_new[1]=targetvel[1]+0.02;
		elseif ret.lbutton[2]==1 then
			targetvel_new[1]=targetvel[1]-0.02;
		elseif ret.lbutton[3]==1 then
				targetvel_new[2]=targetvel[2]+0.02;
		elseif ret.lbutton[4]==1 then
				targetvel_new[2]=targetvel[2]-0.02;
		elseif ret.lstick[2]>600 then
			targetvel_new[3]=targetvel[3]+0.1
		elseif ret.lstick[2]<-600 then
			targetvel_new[3]=targetvel[3]-0.1
		end 
	 	local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
	  if vel_diff>0 then
	    targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
	    print(string.format("Target velocity: %.3f %.3f %.3f",unpack(targetvel)))
			update_display_msg(2, string.format("Vel: %3i %3i %3i",targetvel[1]*100,targetvel[2]*100,targetvel[3]*100) )
	    mcm.set_walk_vel(targetvel)
	  end --end vel update
	elseif display_mode==2 then
		if ret.rbutton==4 then 
			display_mode = 1
			return
		elseif ret.rbutton==1 then
			body_ch:send'init'	
			return
		elseif ret.rbutton==2 then
			body_ch:send'driveready'	
			return
		elseif ret.rbutton==3 then
			body_ch:send'drive'	
			return
		elseif ret.rbutton==4 then 
			display_mode = 1
			return

		end

	end
end

function process_stick(ret)
	local t = Body.get_time()

	--joystck press handling (continuous)
	if display_mode==1 then

	elseif display_mode==2 then
		headpitch = math.min(1, math.max(-1, ret.lstick[1]/800 ))
		headyaw = math.min(1, math.max(-1, ret.lstick[2]/800 ))
		steer = math.min(1, math.max(-1, -ret.rstick[2]/800 ))
		gas = math.min (1,  math.max(0,-ret.rstick[3]/ 800 ))


		steer_mag = 180*DEG_TO_RAD
		gas_mag = 15*DEG_TO_RAD
	
		head_mag = 90*DEG_TO_RAD

		hcm.set_teleop_steering(steer*steer_mag)

		hcm.set_teleop_drive_head({headyaw*head_mag, headpitch*head_mag})


		if ret.lbutton[1]==1 then
			--go forward for 2 sec
		  hcm.set_teleop_throttle_duration(2)
		  hcm.set_teleop_throttle(gas*gas_mag)
		end

	end	
end	


function update_conmmand()
	local t= Body.get_time()
	ret = estop.update()

	if ret.estop~=0 then
		print("ESTOP!!!!!!!!!!",ret.estop)
		--estop pressed, stop
		if Config.estop_mode and Config.estop_mode>0 then

		body_ch:send'estop'
		hcm.set_teleop_estop(1)
		end
		update_display_msg(1,"ESTOP!!!")
		update_display_msg(2,"ESTOP!!!")
		update_display_msg(3,"ESTOP!!!")
		update_display_msg(4,"ESTOP!!!")
		display_mode = 0
		return
	else
	  hcm.set_teleop_estop(0)
	end

	if display_mode==0 then 
		display_mode=1 
		return
	end

	process_click(ret)
	process_stick(ret)
end


t0 = Body.get_time()
estop.init()

update_display_msg(1, " ")
update_display_msg(2, "  ")
update_display_msg(3, "  ")
update_display_msg(4, "  ")
for i=1,10 do
	refresh_display()
	unix.usleep(1e6*command_update_interval)
end

while true do	
	update_conmmand()
	update_display()
	refresh_display()
	unix.usleep(1e6*command_update_interval)
end

estop.close()
