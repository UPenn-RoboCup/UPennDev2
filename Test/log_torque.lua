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

tDelay = 0.001*1E6


local t,t_old = 0,0

local maxtorque = {24.8,44.2, 44.2,88.4,44.2,44.2}

while running do	
	while t<=t_old do
		unix.usleep(tDelay);
		t_old=t
		t=mcm.get_status_t()				
	end
	t_old=t
	local lT = Body.get_lleg_current()
	local rT = Body.get_rleg_current()


	local st = string.format("%.3f %.2f %.2f %.2f %.2f %.2f %.2f",
		t,
		math.max(math.abs(lT[1]),math.abs(rT[1]))/maxtorque[1],
		math.max(math.abs(lT[2]),math.abs(rT[2]))/maxtorque[2],
		math.max(math.abs(lT[3]),math.abs(rT[3]))/maxtorque[3],
		math.max(math.abs(lT[4]),math.abs(rT[4]))/maxtorque[4],
		math.max(math.abs(lT[5]),math.abs(rT[5]))/maxtorque[5],
		math.max(math.abs(lT[6]),math.abs(rT[6]))/maxtorque[6])
	print(st)
end