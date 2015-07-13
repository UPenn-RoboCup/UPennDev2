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
local lstream = Config.net.streams.label
local udp_label_ch = ENABLE_NET and lstream and lstream.udp and si.new_sender(operator, lstream.udp)
local label_ch = lstream and lstream.sub and si.new_publisher(lstream.sub)
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
local dt_monitor = 1 / hz_monitor
local t_send = -math.huge

local ptable = require'util'.ptable
local function update(meta, img)
	--[[
	print('\n=================')
	ptable(meta)
	print()
	--]]

	local Image, detection = Vision.update(meta, img)
	--[[
	if detection.ball then
		print('\n=Ball=')
		ptable(detection.ball)
	--elseif detection.debug.ball then
--		print(detection.debug.ball)
	end
	--]]
	----[[
	if detection.posts then
		print('\n=Posts=')
		ptable(detection.posts)
	--elseif detection.debug.ball then
--		print(detection.debug.ball)
	end
	--]]
	--[[
	print('\n=Obstacles=')
	if detection.obstacles then
		print('Obstacles found!')
		ptable(detection.obstacles)
	elseif detection.debug.obstacles then
		print('No Obstacles...')
		print(detection.debug.obstacle)
	end
	--]]
	--[[
	if detection.line then
		print('\n=Line=')
		ptable(detection.line)
		for i,v in ipairs(detection.line.endpoint) do
			print('line', i, unpack(v))
		end
	--elseif detection.debug.ball then
--		print(detection.debug.ball)
	end
	--]]

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
	vision_ch:send(detection_msg)
	label_ch:send(lA_msg)

	--[[
	local lB_raw = c_zlib(Image.labelB_d, ffi.sizeof(Image.labelB_d))
  local lB_meta = {
    w = Image.wb,
    h = Image.hb,
    sz = #lB_raw,
    c = 'zlib',
    id = 'labelB',
  }
	local lB_msg = {mpack(lB_meta), lB_raw}
	label_ch:send(lB_msg)
	--]]

	-- TODO: How often to send over UDP?
	if udp_ch then
		--udp_ch:send(detection_msg)
		udp_label_ch:send(table.concat(lA_msg))
		if lB_msg then label_udp_ch:send(table.concat(lB_msg)) end
	end
end

local function entry()
	print('vw entry...')
	Vision.entry(metadata)
end
local function exit()
	Vision.exit()
end

-- If required from Webots, return the table
if ... and type(...)=='string' and not tonumber(...) then
	return {entry=entry, update=update, exit=exit}
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
