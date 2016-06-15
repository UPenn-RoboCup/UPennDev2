#!/usr/bin/env luajit
local ENABLE_NET = true
local ENABLE_LOG = false
---------------------------
-- World Manager --
-- (c) Stephen McGill 2014    --
---------------------------
dofile'../include.lua'
local Body = require('Body')
local lW = require'libWorld'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local jpeg = require'jpeg'
local Body = require'Body'
require'hcm'
require'rcm'

local get_time = Body.get_time

local t_log = -math.huge
local LOG_INTERVAL = 1/5

local libLog, logger

local stream = Config.net.streams['world0']
local world_ch = stream and stream.sub and si.new_publisher(stream.sub)


local nlog = 0
local udp_ret, udp_err, udp_data
local t0 = get_time()
local t_debug = 0

--
local buffer = {}
local hz_buffer = 1
local dt_buffer = 1/hz_buffer
local nbuffer = 1
--
local hz_open_send = 1
local dt_open_send = 1/hz_open_send
--
local hz_outdoor_send = 1
local dt_outdoor_send = 1/hz_outdoor_send
--
local hz_indoor_send = 1
local dt_indoor_send = 1/hz_indoor_send
--
local t_buffer = -math.huge
local t_send = -math.huge





local function check_send(msg)
	local is_indoors = 0
	local t = Body.get_time()

	-- Check the buffer
	local dt_buffer0 = t - t_buffer
	if is_indoors>0 and dt_buffer0 > dt_buffer then
		t_buffer = t
		table.insert(buffer, 1, msg)
		if #buffer>nbuffer then table.remove(buffer) end
	end
	if is_indoors==0 then
		buffer = {msg}
	end

	-- Check the sending
	local dt_send0 = t - t_send
	if is_indoors==0 and dt_send0 < dt_outdoor_send then return end
	if is_indoors>0 and dt_send0 < dt_indoor_send then return end
	t_send = t

	for i,m in ipairs(buffer) do

		if world_ch then 
			--print('sending world')

			world_ch:send(m) 
		end

	end

end


local function update()


local c_meta = {
	-- Required for rendering
	id = 'world',
	mrrt = rcm.get_RRT_finished()
}

	c_meta.t = t
	c_meta.n = cnt
	local c_img = '7'
	c_meta.sz = #c_img
	local msg = {mp.pack(c_meta), c_img}
	check_send(msg)

end


if ... and type(...)=='string' then
	TIMEOUT = 0
	return {entry=lW.entry, update=update, exit=lW.exit}
end

local signal = require'signal'
local function shutdown()
	running = false
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local t0 = get_time()
local t_debug = 0
local debug_interval = 1

lW.entry()
running = 1
while running do
	update()
	local t = get_time()

	if t - t_debug > debug_interval then
		t_debug = t
		print(string.format('World | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
--		print('Pose:', vector.pose(wcm.get_robot_pose()))
	end
end
lW.exit()



--[[
#!/usr/bin/env luajit
local ENABLE_NET = true
local ENABLE_LOG = false
---------------------------
-- World Manager --
-- (c) Stephen McGill 2014    --
---------------------------
dofile'../include.lua'
local Body = require('Body')
local lW = require'libWorld'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local jpeg = require'jpeg'
local Body = require'Body'
require'hcm'
require'rcm'

local get_time = Body.get_time

local t_log = -math.huge
local LOG_INTERVAL = 1/5

local libLog, logger

local stream = Config.net.streams['world0']
local world_ch = stream and stream.sub and si.new_publisher(stream.sub)


local nlog = 0
local udp_ret, udp_err, udp_data
local t0 = get_time()
local t_debug = 0

--
local buffer = {}
local hz_buffer = 1
local dt_buffer = 1/hz_buffer
local nbuffer = 1
--
local hz_open_send = 1
local dt_open_send = 1/hz_open_send
--
local hz_outdoor_send = 1
local dt_outdoor_send = 1/hz_outdoor_send
--
local hz_indoor_send = 1
local dt_indoor_send = 1/hz_indoor_send
--
local t_buffer = -math.huge
local t_send = -math.huge





local function check_send(msg)
	local is_indoors = 0
	local t = Body.get_time()

	-- Check the buffer
	local dt_buffer0 = t - t_buffer
	if is_indoors>0 and dt_buffer0 > dt_buffer then
		t_buffer = t
		table.insert(buffer, 1, msg)
		if #buffer>nbuffer then table.remove(buffer) end
	end
	if is_indoors==0 then
		buffer = {msg}
	end

	-- Check the sending
	local dt_send0 = t - t_send
	if is_indoors==0 and dt_send0 < dt_outdoor_send then return end
	if is_indoors>0 and dt_send0 < dt_indoor_send then return end
	t_send = t

	for i,m in ipairs(buffer) do

		if world_ch then world_ch:send(m) end

	end

end


local function update()


local c_meta = {
	-- Required for rendering
	id = 'world',
	mrrt = rcm.get_RRT_finished()
}

	c_meta.t = t
	c_meta.n = cnt
	local c_img = '7'
	c_meta.sz = #c_img
	local msg = {mp.pack(c_meta), c_img}
	check_send(msg)

end


if ... and type(...)=='string' then
	TIMEOUT = 0
	return {entry=lW.entry, update=update, exit=lW.exit}
end

local signal = require'signal'
local function shutdown()
	running = false
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)


lW.entry()
running = 1
while running do
	update()
	if t - t_debug > debug_interval then
		t_debug = t
		print(string.format('World | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
		print('Pose:', vector.pose(wcm.get_robot_pose()))
	end
end
lW.exit()
]]--