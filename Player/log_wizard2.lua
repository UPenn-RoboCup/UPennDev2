---------------------------------
-- Logging manager for Team THOR
-- (c) Stephen McGill 2013
---------------------------------

-- Set the path for the libraries
dofile'../include.lua'
local Config = require'Config'
local Body = require'Body'
require 'unix'
local simple_ipc = require'simple_ipc'
local util = require'util'

---------------------------------
-- Shared Memory
require'jcm'
require'hcm'
require'vcm'
local head_camera, lwrist_camera, joint_positions
---------------------------------

---------------------------------
-- Logging and Replaying set up
local loggers = {}
local log_dir = 'Log/'
local open_logfile = function(dev_name)
  -- Set up log file
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local raw_filename  = string.format('%s/%s_%s_raw.log',log_dir,dev_name,filetime)
  local meta_filename = string.format('%s/%s_%s_meta.log',log_dir,dev_name,filetime)
  return io.open(meta_filename,'w'), io.open(raw_filename,'w')
end

---------------------------------
-- Input Channels
local channel_polls
-- 100Hz joint recording
local channel_timeout = 10 -- ms
local wait_channels = {}

------------------------------------------------------
-- Camera callbacks
------------------------------------------------------
local function head_camera_cb()
	-- Grab the metadata
	local meta, has_more = head_camera.sub:receive()
  -- Grab the compressed image
	local img,  has_more = head_camera.sub:receive()
  -- Write log file
  head_camera.meta_file:write( meta )
  head_camera.raw_file:write( img )
end
--
local function lwrist_camera_cb()
	-- Grab the metadata
	local meta, has_more = lwrist_camera.sub:receive()
  -- Grab the compressed image
	local img,  has_more = lwrist_camera.sub:receive()
  -- Write log file
  lwrist_camera.file:write( meta )
  lwrist_camera.file:write( img )
end

------------------------------------------------------
-- Logger for imu
------------------------------------------------------
local function imu_logger()
	-- Grab the data
	local imu = {}
	imu.name = 'imu'
  imu.t = Body.get_time()
  -- TODO: use shm instead of body?
  imu.rpy = Body.get_sensor_rpy()
  imu.gyro = Body.get_sensor_gyro()

  -- Write log file
  logfile:write( mp.pack(imu) )
end

------------------------------------------------------
-- Logger for sensed joint position and velocity
------------------------------------------------------
local function joint_sensor_logger()
	-- Grab the data
	local joint = {}
	joint.name = 'sensor'
  joint.t = Body.get_time()
  joint.pos = jcm.get_sensor_position()
  joint.vel = jcm.get_sensor_velocity()

  -- Write log file
  logfile:write( mp.pack(joint) )
end

------------------------------------------------------
-- Logger for commanded joint position and velocity
------------------------------------------------------
local function joint_actuator_logger()
	-- Grab the data
	local joint = {}
	joint.name = 'actuator'
  joint.t = Body.get_time()
  joint.pos = jcm.get_actuator_command_position()
  joint.vel = jcm.get_actuator_command_velocity()

  -- Write log file
  logfile:write( mp.pack(joint) )
end

------------------------------------------------------
-- Logger for FSR sensor
------------------------------------------------------
local function fsr_logger()
  local fsr = {}
  fsr.name = 'fsr'
  fsr.lfoot = jcm.get_sensor_lfoot()
  fsr.rfoot = jcm.get_sensor_rfoot()
  -- Write
  logfile:write( mp.pack(fsr) )
end

local setup_log = {
  head_camera = function()
    head_camera = {}
    head_camera.sub = simple_ipc.new_subscriber'head_camera'
    head_camera.sub.callback = head_camera_cb
    table.insert( wait_channels, head_camera.sub )
    return head_camera
  end,
  lwrist_camera = function()
    lwrist_camera = {}
    lwrist_camera.sub = simple_ipc.new_subscriber'lwrist_camera'
    lwrist_camera.sub.callback = lwrist_camera_cb
    table.insert( wait_channels, lwrist_camera )
    return lwrist_camera
  end
}

-- Close the camera properly upon Ctrl-C
local signal = require 'signal'
local function shutdown()
  print'Shutting down the Log files...'
  for _,logger in ipairs(loggers) do
    logger.meta_file:close()
    logger.raw_file:close()
    print('Closed log',logger.name)
  end
  print'Done!'
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local log = {}
function log.entry()
	
	-- Set up listeners based on the input arguments
	for _,name in ipairs(arg) do
    local setup = setup_log[name]
    if type(setup)=='function' then
      local logger = setup()
      logger.meta_file, logger.raw_file = open_logfile(name)
      logger.name = name
      table.insert(loggers,logger)
    end
	end
  
  -- Set up the channels
  channel_polls = simple_ipc.wait_on_channels( wait_channels )
end

local t0 = unix.time()
local t_debug = unix.time()
local DEBUG_INTERVAL = 1
function log.update()
  ------------------
  -- Perform the poll
  local npoll = channel_polls:poll(channel_timeout)
  local t = unix.time()
  ------------------
  -- Log things not on the poll at a fixed 100Hz rate
  --[[
  imu_logger()
  joint_sensor_logger()
  joint_actuator_logger()
  fsr_logger()
  --]]
  local t_diff = t-t_debug
  if t_diff>DEBUG_INTERVAL then
    t_debug = t
    print(string.format('%d seconds of logs...',t-t0))
  end
end

function log.exit()
	shutdown()
end

-- Main loop
print'Beginning to log...'
log.entry()
while true do log.update() end
log.exit()

return log
