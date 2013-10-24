---------------------------------
-- Logging manager for Team THOR
-- (c) Qin He, Stephen McGill 2013
---------------------------------

-- Set the path for the libraries
dofile'../include.lua'
local Config = require'Config'
local Body = require'Body'
require 'unix'

---------------------------------
-- Libraries
local simple_ipc = require'simple_ipc'
local carray = require'carray'
local mp = require'msgpack'
local jpeg = require'jpeg'
local zlib = require'zlib'
local util = require'util'
local vector = require'vector'
local libLaser = require'libLaser'
---------------------------------

-- TODO: contact sensor
-- TODO: hcm inputs

---------------------------------
-- Shared Memory
require'jcm'
require'hcm'
require'vcm'
local head_camera, lwrist_camera
---------------------------------

---------------------------------
-- Logging and Replaying set up
local log_dir = 'Log/'
local open_logfile = function(dev_name)
  -- Set up log file
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local filename = string.format('%s/%s_%s.log',log_dir,dev_name,filetime)
  return io.open(filename,'w')  
end

---------------------------------

-- Input Channels
local channel_polls
-- 100Hz joint recording
local channel_timeout = 10 -- ms
local wait_channels = {}

-- Lidar objects
local hlidar -- head
local clidar -- chest

---------------------------------
-- Filter Parameters
local l0minFOV = -135*Body.DEG_TO_RAD
local l0maxFOV =  135*Body.DEG_TO_RAD
local l1minFOV = -45*Body.DEG_TO_RAD 
local l1maxFOV =  45*Body.DEG_TO_RAD 
local l0minHeight = -0.6 -- meters
local l0maxHeight = 1.2
-- We don't need height limits for chest lidar
local minRange = 0.15 -- meters
local maxRange = 28
if IS_WEBOTS then
  maxRange = 9.5
end


---------------------------------
-- Callbacks for receiving lidar readings
local function head_callback()
  --print('HEAD CALLBACK')
  -- Grab the data  
  local meta, has_more = head_lidar_ch:receive()
  local metadata = mp.unpack(meta)
   	
	metadata.name = 'headlidar'
  -- Get raw data from shared memory
  --metadata.ranges = vcm.get_head_lidar_scan()

  ---[[
  -- TODO: May try to put into the lidar message itself
  -- which is useful for a separate computer to perform slam
  local ranges = Body.get_head_lidar()
  --print('lidar sizes',#ranges,hlidar.ranges:size(1))
  -- Copy of ranges for use in libLaser
  ranges:tensor( hlidar.ranges )

  -- Take log
	-- torch is easier to be logged...
	metadata.ranges = hlidar.ranges
	--]]

	logfile:write( mp.pack(metadata) )
 
end



------------------------------------------------------
-- Chest lidar callback
------------------------------------------------------
local function chest_callback()

  -- Grab the data
  local meta, has_more = chest_lidar_ch:receive()
  local metadata = mp.unpack(meta)

	-- Get raw data from shared memory
	local ranges = Body.get_chest_lidar()
	--print('Lidar1 range size:', clidar.ranges:size(1))
	-- Copy of ranges for use in libLaser
	ranges:tensor( clidar.ranges )
  
  -- Take log
	-- torch is easier to be logged...
	metadata.name = 'chestlidar'
	metadata.ranges = clidar.ranges
	logfile:write( mp.pack(metadata) )
  
end

------------------------------------------------------
-- Mesh callback
------------------------------------------------------
local function mesh_callback()
	--print('logging mesh!!')
	-- Grab the data
	local meta, has_more = mesh_ch:receive()

  -- Write log file
  logfile:write( meta )
end

------------------------------------------------------
-- Camera callback
------------------------------------------------------
local function head_camera_cb()
	-- Grab the metadata
	local meta, has_more = head_camera.sub:receive()
  -- Grab the compressed image
	local img,  has_more = head_camera.sub:receive()
  -- Write log file
  head_camera.file:write( meta )
  head_camera.file:write( img )
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


local log = {}

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

function log.entry()
	
	-- Set up listeners based on the input arguments
	for _,name in ipairs(arg) do
    local setup = setup_log[name]
    if type(setup)=='function' then
      local logger = setup()
      logger.file = open_logfile(name)
    end
	end
  
  -- Set up the channels
  channel_polls = simple_ipc.wait_on_channels( wait_channels )
end

function log.update()
  ------------------
  -- Perform the poll
  local npoll = channel_polls:poll(channel_timeout)
  ------------------
  -- Log things not on the poll at a fixed 100Hz rate
  --[[
  imu_logger()
  joint_sensor_logger()
  joint_actuator_logger()
  fsr_logger()
  --]]
end

function log.exit()
	logfile:close()
end

-- Main loop
print'Beginning to log...'
log.entry()
while true do log.update() end
log.exit()

return log
