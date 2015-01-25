#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local t_last = Body.get_time()
local tDelay = 0.005*1E6




local getch = require'getch'
local running = true
local key_code

local sformat = string.format


require'mcm'

while running do
	local forceZ = mcm.get_status_forceZ()
	local forceTotal = mcm.get_status_forceTotal()
	local uZMP = mcm.get_status_uZMP()
	local uZMPMeasured = mcm.get_status_uZMPMeasured()

	local LZMP = mcm.get_status_LZMP()
	local RZMP = mcm.get_status_RZMP()

	os.execute('clear')
	print(sformat("force Z: %d %d",forceZ[1],forceZ[2]))
	print(sformat("force Total: %d %d",forceTotal[1],forceTotal[2]))

--	print(sformat("Torque L: %.2f %.2f R: %.2f %.2f",forceTotal[1],forceTotal[2]))

	print(sformat("uZMP: %.1f %.1f / uZMP(M): %.1f %.1f",
		uZMP[1]*100, uZMP[2]*100, uZMPMeasured[1]*100,	uZMPMeasured[2]*100

		))

	print(sformat("ZMP err: L %.1f %.1f R %.1f %.1f cm",
		LZMP[1]*100, LZMP[2]*100,RZMP[1]*100, RZMP[2]*100))

	unix.usleep(tDelay);

end
