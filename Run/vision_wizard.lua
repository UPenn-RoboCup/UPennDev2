#!/usr/bin/env luajit
local ENABLE_NET = true
local ENABLE_LOG = false
-----------------------------------
-- Camera manager
-- (c) Stephen McGill, 2014
-----------------------------------
dofile'../include.lua'
if type(arg)~='table' then IS_WEBOTS=true end
local si = require'simple_ipc'
local munpack = require'msgpack.MessagePack'.unpack
local mpack = require'msgpack.MessagePack'.pack
local jpeg = require'jpeg'
local Body = require'Body'
local get_time = Body.get_time
local Vision = require'Vision'
local c_zlib = require'zlib.ffi'.compress
local ok, ffi = pcall(require,'ffi')

-- Grab the metadata for this camera
local metadata
local camera_id
if type(arg)~='table' or not arg[1] then
	-- TODO: Find the next available camera
	camera_id = 1
	metadata = Config.camera[1]
else
	camera_id = tonumber(arg[1])
	if camera_id then
		metadata = assert(Config.camera[camera_id], 'Bad Camera ID')
	else
		for id, c in ipairs(Config.camera) do
			if arg[1] == c.name then
				camera_id = id
				metadata = c
				break
			end
		end
		assert(metadata, 'Bad camera name')
	end
end

local t_log = -math.huge
local LOG_INTERVAL = 1/5

local libLog, logger

-- Extract metadata information
local w = metadata.w
local h = metadata.h
local name = metadata.name
-- Who to send to
local operator
--operator = Config.net.operator.wireless
operator = Config.net.operator.wired
-- Network Channels/Streams
local vision_identifier = 'vision'..(camera_id-1)
local camera_identifier = 'camera'..(camera_id-1)
local stream = Config.net.streams[vision_identifier]
local udp_ch = ENABLE_NET and stream and stream.udp and si.new_sender(operator, stream.udp)
local vision_ch = stream and stream.sub and si.new_publisher(stream.sub)
--
print('Vision | ', operator, vision_identifier, stream.udp, udp_ch)

-- LOGGING
if ENABLE_LOG then
	libLog = require'libLog'
	logger = libLog.new('vision', true)
end
--
local t0 = get_time()
local t_debug = 0
--
local hz_monitor = 1
local dt_monitor = 1/hz_monitor
local t_send = -math.huge

local ptable = require'util'.ptable
local function update(meta, img)
	--ptable(meta)

	local Image, detection = Vision.update(meta, img)
	-- Send labelA and detection information
	local lA_raw = c_zlib(Image.labelA_d, ffi.sizeof(Image.labelA_d))
  local lA_meta = {
    w = Image.wa,
    h = Image.ha,
    sz = #lA_raw,
    c = 'zlib',
    id = 'labelA',
  }
	local lA_msg = {mpack(lA_meta), lA_raw}
	local detection_msg = mpack(detection)
	vision_ch:send(lA_msg)
	vision_ch:send(detection_msg)
	if udp_ch then
		udp_ch:send(table.concat(lA_msg))
		udp_ch:send(detection_msg)
	end
end

local function entry()
	Vision.entry(metadata)
end
local function exit()
	Vision.exit()
end

-- If required from Webots, return the table
if ... and type(...)=='string' and not tonumber(...) then
	return {entry=nil, update=update, exit=nil}
end

local poller
local function cb(skt)
	local idx = poller.lut[skt]
	local mdata, raw = unpack(skt:recv_all())
	local meta = munpack(mdata)
	update(meta, raw)
end

local camera_ch = si.new_subscriber(camera_identifier)
camera_ch.callback = cb
poller = si.wait_on_channels({camera_ch})

-- Cleanly exit on Ctrl-C
local signal = require'signal'.signal
local running = true
local function shutdown()
	print('Shutdown!\n')
	poller:stop()
	running = false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

entry()
poller:start()
exit()
