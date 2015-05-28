#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

estop = require'estop'

targetvel={0,0,0}
targetvel_new={0,0,0}








function update_estop()
	ret = estop.update()


	if ret.estop==1 then
		print("ESTOP!!!!!!!!!!")
		--estop pressed, stop
		body_ch:send'stop'
	else
		if ret.rbutton==1 then
			print("INIT!!!!!!")
			body_ch:send'init'	
		elseif ret.rbutton==3 then
			motion_ch:send'hybridwalk'


		elseif ret.lbutton[1]==1 then
			--targetvel_new[1]=targetvel[1]+0.02;
			targetvel={0.06,0,0}
		elseif ret.lbutton[2]==1 then
			targetvel={-0.04,0,0}
		elseif ret.lbutton[3]==1 then
			targetvel={0,0.04,0}
		elseif ret.lbutton[4]==1 then
			targetvel={0,-0.04,0}
		end

    mcm.set_walk_vel(targetvel)
	end
end


estop.init()

while true do
	update_estop()
	unix.usleep(1e6*0.2)
end

estop.close()