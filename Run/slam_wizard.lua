#!/usr/bin/env luajit
local ENABLE_LOG = false
-- Mesh Wizard for Team THOR
-- Accumulate lidar readings into an image for mesh viewing
-- (c) Stephen McGill, Seung Joon Yi, 2013, 2014
dofile'../include.lua'
local libMesh = require'libMesh'
local si = require'simple_ipc'
local mpack = require'msgpack.MessagePack'.pack
local munpack = require('msgpack.MessagePack')['unpack']
local Body = require'Body'
local ffi = require'ffi'
require'vcm'
require'hcm'

local sin, cos = require'math'.sin, require'math'.cos
local T = require'Transform'

local pillar_ch = si.new_publisher('pillars')

local polar_interval = 15 * DEG_TO_RAD
local function find_pillars(xyz, polar)
	local xyz_com, xyz_world = xyz[1], xyz[2]
	local rho, theta = unpack(polar)
	local pillars = {}
	local interval = polar_interval + theta[1]
	local xymin
	local rmin = math.huge
	for i, a in ipairs(theta) do
		local xyz = xyz_com[i]
		local xyzw = xyz_world[i]
		local r = rho[i]
		if a<interval then
			-- Filter too close, too far and ground
			if r<rmin and r>0.24 and r<5 and xyzw[3]>0 then
				xymin = {xyz[1], xyz[2]}
				rmin = r
--			elseif not xymin then
--				rmin = math.huge
--				xymin = {xyz[1], xyz[2]}
			end
		else
			if xymin then table.insert(pillars, xymin) end
			xymin = nil
			rmin = math.huge
			interval = interval + polar_interval
		end
	end
	if xymin then table.insert(pillars, xymin) end
	pillar_ch:send(mpack(pillars))
	--[[
	print('Sending pillars', #pillars)
	for i,p in ipairs(pillars) do
		print(p[2], p[1])
	end
	--]]
end

local Thead = T.trans(0,0,0.282)
local function head3d(meta, scan)
	local scan_fl = ffi.cast('float*', scan)
	local mid = meta.n / 2 * meta.res
	local angles, rho = {}, {}
	for i=1,meta.n do
		table.insert(angles, i * meta.res - mid)
		table.insert(rho, scan_fl[i-1])
	end
	local xyz = {}
	for i,a in ipairs(angles) do
		table.insert(xyz, {rho[i] * cos(a), rho[i] * sin(a), 0.1})
	end
	local xyz_actuated = {}
	local Tact = T.rotY(meta.angle[2]) * T.rotZ(meta.angle[1])
	for i,p in ipairs(xyz) do
		table.insert(xyz_actuated, Tact * p)
	end
	local xyz_head = {}
	for i,p in ipairs(xyz_actuated) do
		table.insert(xyz_head, Thead * p)
	end
	local xyz_com = {}
	local Twaist = T.rotZ(meta.qWaist[1])
	for i, p in ipairs(xyz_head) do
		table.insert(xyz_com, Twaist*p)
	end
	local rho_com = {}
	local theta_com = {}
	for i, p in ipairs(xyz_com) do
		table.insert(theta_com, math.atan2(p[2], p[1]))
		table.insert(rho_com, math.sqrt(math.pow(p[2],2), math.pow(p[1],2)))
	end

	local Tworld = T.from_flat(meta.tfG16)
	local xyz_world = {}
	for i, p in ipairs(xyz_com) do
		table.insert(xyz_world, Tworld*p)
	end
	return {xyz_com, xyz_world}, {rho_com, theta_com}
end

local Tchest = T.trans(0.05, 0, 0.09)
local function chest3d(meta, scan)
	local scan_fl = ffi.cast('float*', scan)
	local xyz = {}
	for i,a in ipairs(angles) do
		local r = scan_fl[i] + 0.02
		table.insert(xyz, {r * sin(a), r * cos(a), 0})
	end
	local Tcom = Tchest * T.rotZ(meta.a) * T.rotX(math.pi/2)
	local xyz_com = {}
	for i, p in ipairs(xyz) do
		table.insert(xyz_com, Tcom*p)
	end
	local Tworld = T.from_flat(meta.tfG16)
	local xyz_world = {}
	for i, p in ipairs(xyz_com) do
		table.insert(xyz_world, Tworld*p)
	end
	return xyz_world, xyz_com
end

local function entry()

end

local np = 0
local function update(meta, scan)
	local points
	-- Form 3D coordinates
	if meta.id=='lidar0' then
		--points = chest3d(meta, scan)
	elseif meta.id=='lidar1' then
		local xyz, polar = head3d(meta, scan)
		find_pillars(xyz, polar)
	end

	--[[
	if np<1 then
		for i,p in ipairs(points) do
			print(i, unpack(p))
		end
	end
	--]]
	np = np + 1
end

local function exit()
end

-- If required from Webots, return the table
if ... and type(...)=='string' then
	return {entry=entry, update=update, exit=exit}
end

local poller
local function cb(skt)
	local idx = poller.lut[skt]
	local mdata, ranges = unpack(skt:recv_all())
	local meta = munpack(mdata)
	update(meta, ranges)
end

local lidar0_ch = si.new_subscriber'lidar0'
local lidar1_ch = si.new_subscriber'lidar1'

lidar0_ch.callback = cb
lidar1_ch.callback = cb
poller = si.wait_on_channels({lidar1_ch})

-- Cleanly exit on Ctrl-C
local signal = require'signal'.signal
local running = true
local function shutdown()
	io.write('Shutdown!\n')
	poller:stop()
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

poller:start()

if ENABLE_LOG then
	logger0:stop()
	logger1:stop()
end
