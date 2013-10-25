---------------------------------
-- Logging manager for Team THOR
-- (c) Stephen McGill, Qin He 2013
---------------------------------

-- Set the path for the libraries
dofile'../include.lua'
local Config = require'Config'
local Body = require'Body'
require 'unix'
local simple_ipc = require'simple_ipc'
local util = require'util'
local mp = require'msgpack'

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
local open_logfile = function(dev_name,only_meta)
  -- Set up log file
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local meta_filename = string.format('%s/%s_%s_meta.log',LOG_DIR,dev_name,filetime)
  if not only_meta then
    local raw_filename  = string.format('%s/%s_%s_raw.log',LOG_DIR,dev_name,filetime)
    return io.open(meta_filename,'w'), io.open(raw_filename,'w')
  else
    return io.open(meta_filename,'w')
  end
end

---------------------------------
-- Input Channels
local pulse_sub   = simple_ipc.new_subscriber'pulse'
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
  lwrist_camera.meta_file:write( meta )
  lwrist_camera.raw_file:write( img )
end

------------------------------------------------------
-- Logger for the body
------------------------------------------------------
local body_log_file
local function body_logger()
	-- Grab the data
	local body = {}
  --Body.get_time()
  body.t = tonumber(pulse_sub:receive())
  body.command_position = jcm.get_actuator_command_position()
  body.twrite = jcm.get_twrite_command_position()
  body.position = jcm.get_sensor_position()
  body.tread = jcm.get_tread_position()
  body.rpy  = Body.get_sensor_rpy()
  body.gyro = Body.get_sensor_gyro()
  body.lfoot = jcm.get_sensor_lfoot()
  body.rfoot = jcm.get_sensor_rfoot()

  -- Write log file
  body_log_file:write( mp.pack(body) )
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
    table.insert( wait_channels, lwrist_camera.sub )
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
  body_log_file:close()
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
  
  -- Body logging on a pulse from state_wizard's Body.update()
  body_log_file     = open_logfile('body',true)
  pulse_subcallback = body_logger
  table.insert( wait_channels, pulse_sub )
  
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
  
  -- Debug how long we've been logging
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
