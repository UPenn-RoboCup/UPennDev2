#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local RAD_TO_DEG= 180/math.pi
local sformat = string.format


local qWaist = Body.get_waist_position()
local qLArm = Body.get_larm_position()
local qRArm = Body.get_rarm_position()


print("Waist yaw:",qWaist[1]*RAD_TO_DEG)
print(string.format( "LWRIST err: %d %d %d",
	qLArm[5]*RAD_TO_DEG - 90,
	qLArm[6]*RAD_TO_DEG,
	qLArm[7]*RAD_TO_DEG+90)
	)

print(string.format( "RWRIST err: %d %d %d",
	qRArm[5]*RAD_TO_DEG + 90,
	qRArm[6]*RAD_TO_DEG,
	qRArm[7]*RAD_TO_DEG - 90)
)

