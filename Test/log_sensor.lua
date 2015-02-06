#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local t_last = Body.get_time()
local tDelay = 0.1*1E6



local RAD_TO_DEG= 180/math.pi


local getch = require'getch'
local running = true
local key_code

local sformat = string.format

require'mcm'
require'hcm'
require'wcm'

count = 0

while running do
	local lft = mcm.get_status_LFT()
	local rft = mcm.get_status_RFT()
	local imu = mcm.get_status_IMU()

	local uZMP = mcm.get_status_uZMP()
	local uZMPMeasured = mcm.get_status_uZMPMeasured()

	local LZMP = mcm.get_status_LZMP()
	local RZMP = mcm.get_status_RZMP()


	local aShiftX = mcm.get_walk_aShiftX()
	local aShiftY = mcm.get_walk_aShiftY()

	local zLeg = mcm.get_status_zLeg()


  	local enable_balance = hcm.get_legdebug_enable_balance()

  	local angleShift = mcm.get_walk_angleShift()

	local pose = wcm.get_robot_pose()


	count = count + 1

	if count%2 ==0 then
		os.execute('clear')

		print(sformat("Balancing: P Knee %.1f Ankle %.1f / R hip %.1f ankle %.1f",
			RAD_TO_DEG*angleShift[3], RAD_TO_DEG*angleShift[1]  , RAD_TO_DEG*angleShift[4], RAD_TO_DEG*angleShift[2]))

		print(sformat("FZ: %d %d PT: %.1f %.1f RT: %.1f %.1f",
			lft[1],rft[1], lft[3],rft[3], lft[2],rft[2]
			))
		print(sformat("uZMPTarget: %.1f %.1f / uZMP: %.1f %.1f",
			uZMP[1]*100, uZMP[2]*100, uZMPMeasured[1]*100,	uZMPMeasured[2]*100

			))
		print(sformat("ZMP err: L %.1f %.1f R %.1f %.1f cm",
			LZMP[1]*100, LZMP[2]*100,RZMP[1]*100, RZMP[2]*100))

		print(sformat("leg balancing: %d %d",enable_balance[1],enable_balance[2]))

		print(sformat("Foot Height: Left %.1f Right %.1f cm",
		zLeg[1]*100,zLeg[2]*100))

		print(sformat("Foot angles: Left P%.1f R%.1f  Right P%.1f R%.1f",
			RAD_TO_DEG*aShiftY[1],RAD_TO_DEG*aShiftX[1],RAD_TO_DEG*aShiftY[2],RAD_TO_DEG*aShiftX[2]            
			))

		print()
		print(sformat("Pose: %.2f %.2f %d(deg)",pose[1],pose[2],pose[3]*180/math.pi))
	end

	unix.usleep(tDelay);

end
