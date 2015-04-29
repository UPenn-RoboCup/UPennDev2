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
require'vcm'
require'hcm'

local libSlam2 = require'libSlam2'
local locale = libSlam2.new_locale()
local sin, cos = require'math'.sin, require'math'.cos

local T = require'Transform'

-- Just hard code for now
local angles = {}
for i=-540,540 do a[i] = i * 0.25 * DEG_TO_RA end

local function head3d(meta, scan)
	
	return xyz
end


local Tchest = T.trans(0.05,0,0.09)
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
		table.insert(xyz_com, Tworld*p)
	end
	return xyz_world
end

local function entry()

end

local function update(meta, scan)
	local points
	-- Form 3D coordinates
	if meta.id=='lidar0' then
		--points = chest3d(meta, scan)
	elseif meta.id=='lidar1' then
		points = head3d(meta, scan)
	end
	if not poitns then return end
	locale:add_scan(points)
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
poller = si.wait_on_channels({lidar_ch})

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