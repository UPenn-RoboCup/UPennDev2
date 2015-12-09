#!/usr/bin/env luajit
dofile'../fiddle.lua'
local Body   = require'Body'
local openni = require 'openni'
local signal = require'signal'
require'vcm'
require'mcm'                                                                    
require'hcm'                                                                    
require'dcm'                                                                    
require'wcm'

openni.startup()

-- Verify stream
local WIDTH, HEIGHT = 320, 240
local DEPTH_NELEMENTS = WIDTH*HEIGHT
local depth_info, color_info = openni.stream_info()
assert(depth_info.width==WIDTH,'Bad depth resolution')
assert(color_info.width==WIDTH,'Bad color resolution')

ENABLE_LOG = true

local libLog, logger
if ENABLE_LOG then
	libLog = require'libLog'
	log_rgb = libLog.new('k_rgb', true)
	log_depth = libLog.new('k_depth', true)
end

-- Set up timing debugging
local cnt = 0;
local t_last = Body.get_time()
local t_debug = 5

function shutdown()
	print'Shutting down the OpenNI device...'
	openni.shutdown()
	os.exit()
end
signal.signal("SIGINT",  shutdown)
signal.signal("SIGTERM", shutdown)

print("Starting")

-- Start loop
while true do
	-- Acquire the Data
	local dframe, cframe = openni.update_rgbd()
	-- Check the time of acquisition
	local t = Body.get_time()

	-- Save the metadata
	local metadata = {}
	metadata.t = t
	metadata.rpy = dcm.get_sensor_rpy()
	metadata.acc = dcm.get_sensor_accelerometer()
	metadata.gyro = dcm.get_sensor_gyro()

	if ENABLE_LOG then
			log_depth:record(metadata, dframe,  320*240*2)
			if log_depth.n >= 100 then
				log_depth:stop()
				print('Open new depth log!')
				log_depth = libLog.new('k_depth', true)
			end
			log_rgb:record(metadata, cframe,  320*240*3)
			if log_rgb.n >= 100 then
				log_rgb:stop()
				print('Open new rgb log!')
				log_rgb = libLog.new('k_rgb', true)
			end
	end

	-- Debug the timing
	cnt = cnt + 1
	if t-t_last > t_debug then
		print(string.format("RGBD: %.2f FPS", cnt/t_debug))
		t_last = t
		cnt = 0
	end

end
