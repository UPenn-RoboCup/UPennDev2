#!/usr/bin/env luajit
-- Hokuyo LIDAR Wizard
-- Reads lidar scans and saves to shared memory
-- (c) Stephen McGill 2013, 2014
dofile'../include.lua'

--local LIDAR_ID = assert(tonumber(arg[1]), 'Need to give an id (0 or 1)')

local ENABLE_LOG = false

local libHokuyo  = require'libHokuyo'
local signal = require'signal'.signal
local get_time = unix.time
local mpack = require'msgpack'.pack
local pose_global = require'util'.pose_global
local color = require'util'.color
local si = require'simple_ipc'
local Body = require'Body'
local T = require'Transform'
require'wcm'
require'mcm'

-- Setup the Hokuyos array
local hokuyos = {}
local streams = Config.net.streams

-- Initialize the Hokuyos
--local h0 = libHokuyo.new_hokuyo('/dev/ttyACM0')
--local h0 = libHokuyo.new_hokuyo('/dev/cu.usbmodem1411',nil,9600)
--local h0 = libHokuyo.new_hokuyo(11)
local h0 = libHokuyo.new_hokuyo(10) -- chest on mk2
h0.name = 'chest'
h0.ch = si.new_publisher(streams.lidar0.sub)
h0.tcp_ch = si.new_publisher(streams.lidar0.tcp)
h0.metadata = {
	id='lidar0'
}
h0.angle = Body.get_lidar_position
----[[
local h1 = libHokuyo.new_hokuyo(11) -- head on mk2
h1.name = 'head'
h1.ch = si.new_publisher(streams.lidar1.sub)
h1.tcp_ch = si.new_publisher(streams.lidar1.tcp)
h1.metadata = {
	id='lidar1'
}
h1.angle = Body.get_head_position
--]]

local libLog, logger, nlog
if ENABLE_LOG then
	libLog = require'libLog'
	h0.logger = libLog.new(h0.name, true)
	h1.logger = libLog.new(h1.name, true)
	nlog = 0
end

local cb = function(self, data)
	local metadata = self.metadata
	metadata.t = get_time()
	metadata.angle = self.angle()

	local rpy = Body.get_rpy()
	local uComp = mcm.get_stance_uTorsoComp()
	uComp[3] = 0
	local torso0 = pose_global(uComp, mcm.get_status_bodyOffset())
	local pose = wcm.get_robot_pose()
	local torsoG = pose_global(torso0, pose)
	local bh = mcm.get_stance_bodyHeight()
	-- TODO: Waist!!!

	metadata.tfL6 = {torso0.x, torso0.y, bh, rpy[1], rpy[2], torso0.a}
	metadata.tfG6 = {torsoG.x, torsoG.y, bh, rpy[1], rpy[2], torsoG.a}
	metadata.tfL16 = T.flatten(T.transform6D(metadata.tfL6))
	metadata.tfG16 = T.flatten(T.transform6D(metadata.tfG6))
	metadata.n = self.n
	metadata.res = self.res
	metadata.rsz = #data

	local send_data = {mpack(metadata), data}
	local ret = self.ch:send(send_data)
	local ret = self.tcp_ch:send(send_data)

	if ENABLE_LOG then
		self.logger:record(metadata, data)
		nlog = nlog + 1
		if nlog % 400 == 0 then
			self.logger:stop()
			self.logger = libLog.new(self.name, true)
			print('Open new log!', self.name)
		end
	end

end


-- Ensure that we shutdown the devices properly
local function shutdown()
	print'Shutting down the Hokuyos...'
	for i,h in ipairs(hokuyos) do
		h:stream_off()
		h:close()
		print('Closed Hokuyo',i)
	end
	if ENABLE_LOG then
		logger:stop()
	end
	os.exit()
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

if h0 then
h0.callback = cb
table.insert(hokuyos, h0)
end
if h1 then
h1.callback = cb
table.insert(hokuyos, h1)
end

-- Begin to service
os.execute('clear')
assert(#hokuyos>0, "No hokuyos detected!")
print( color('Servicing '..#hokuyos..' Hokuyos','green') )

local main = function()
	local main_cnt = 0
	local t0 = get_time()
	while true do
		main_cnt = main_cnt + 1
		local t_now = get_time()
		local t_diff = t_now - t0
		if t_diff>1 then
			local debug_str = string.format('\nMain loop: %7.2f Hz',main_cnt/t_diff)
			debug_str = color(debug_str,'yellow')
			for i,h in ipairs(hokuyos) do
				debug_str = debug_str..string.format(
				'\n\t%s Hokuyo:\t%5.1fHz\t%4.1f ms ago',h.name, 1/h.t_diff, (t_now-h.t_last)*1000)
			end
			os.execute('clear')
			print(debug_str)
			t0 = t_now
			main_cnt = 0
		end
		coroutine.yield()
	end
end

libHokuyo.service( hokuyos, main )
