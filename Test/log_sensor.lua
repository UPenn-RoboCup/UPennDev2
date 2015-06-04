#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local t_last = Body.get_time()
local tDelay = 0.05*1E6



local RAD_TO_DEG= 180/math.pi


local getch = require'getch'
local util = require'util'
local running = true
local key_code

local sformat = string.format

require'mcm'
require'hcm'
require'dcm'
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

	local LZMPr = dcm.get_sensor_lzmp()
	local RZMPr = dcm.get_sensor_rzmp()

	local aShiftX = mcm.get_walk_aShiftX()
	local aShiftY = mcm.get_walk_aShiftY()
	local zLeg = mcm.get_status_zLeg()

 	local enable_balance = hcm.get_legdebug_enable_balance()
 	local angleShift = mcm.get_walk_angleShift()

	local pose = wcm.get_robot_pose()


	local llt =  mcm.get_status_lleg_torque()
	local rlt =  mcm.get_status_rleg_torque()


	local t_max = {45,45,45,90,45,45}

	count = count + 1


	local rpy = dcm.get_sensor_rpy()
	local acc = dcm.get_sensor_accelerometer()
	local gyro = dcm.get_sensor_gyro()


	if count%2 ==0 then
		os.execute('clear')

		print(sformat("Angle: R%.1f P%.1f",
			rpy[1]*RAD_TO_DEG, rpy[2]*RAD_TO_DEG))


		print(sformat("Balancing: P Knee %.1f Ankle %.1f / R hip %.1f ankle %.1f",
			RAD_TO_DEG*angleShift[3], RAD_TO_DEG*angleShift[1]  , RAD_TO_DEG*angleShift[4], RAD_TO_DEG*angleShift[2]))

		print(sformat("FZ: %.1f %.1f PT: %.1f %.1f RT: %.1f %.1f",
			lft[1],rft[1], lft[3],rft[3], lft[2],rft[2]
			))
		print(sformat("uZMPTarget: %.1f %.1f / uZMP: %.1f %.1f",
			uZMP[1]*100, uZMP[2]*100, uZMPMeasured[1]*100,	uZMPMeasured[2]*100

			))

		print(sformat("ZMP err(raw): L %.1f %.1f R %.1f %.1f cm",
			LZMPr[1]*100, LZMPr[2]*100,RZMPr[1]*100, RZMPr[2]*100))


		print(sformat("ZMP err(fil): L %.1f %.1f R %.1f %.1f cm",
			LZMP[1]*100, LZMP[2]*100,RZMP[1]*100, RZMP[2]*100))

		print(sformat("leg balancing: %d %d",enable_balance[1],enable_balance[2]))

		print(sformat("Foot Height: Left %.1f Right %.1f cm",
		zLeg[1]*100,zLeg[2]*100))

		print(sformat("Foot angles: Left P%.1f R%.1f  Right P%.1f R%.1f",
			RAD_TO_DEG*aShiftY[1],RAD_TO_DEG*aShiftX[1],RAD_TO_DEG*aShiftY[2],RAD_TO_DEG*aShiftX[2]            
			))

		print()
		print(sformat("Pose: %.2f %.2f %d(deg)",pose[1],pose[2],pose[3]*180/math.pi))


		print(string.format("LLeg Torque: %s %s %s %s %s %s",
			util.colorcode(llt[1],t_max[1]),
			util.colorcode(llt[2],t_max[2]),
			util.colorcode(llt[3],t_max[3]),
			util.colorcode(llt[4],t_max[4]),
			util.colorcode(llt[5],t_max[5]),
			util.colorcode(llt[6],t_max[6])))
		print(string.format("RLeg Torque: %s %s %s %s %s %s",
			util.colorcode(rlt[1],t_max[1]),
			util.colorcode(rlt[2],t_max[2]),
			util.colorcode(rlt[3],t_max[3]),
			util.colorcode(rlt[4],t_max[4]),
			util.colorcode(rlt[5],t_max[5]),
			util.colorcode(rlt[6],t_max[6])))


	end

	unix.usleep(tDelay);

end
