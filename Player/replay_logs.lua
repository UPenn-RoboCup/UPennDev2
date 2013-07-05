dofile'../include.lua'

-- Run in realtime?
-- TODO: Should be a realtime factor
local REALTIME_FACTOR = 1

-- Data log location
-- TODO: Take from the command line
local LOG_PATH = '~/shadwell/day2_third/'
local dataStamp = '02.27.2013'

------------------------------
-- How to replay which logs
-- ['Label of Log to playback'] = 'Simple IPC channel name'
-- TODO: Automatically generate this, since it should be an ipairs
-- i.e. have a regex parse the directory and have the channel name as the prefix
local replay_mappings = {}
replay_mappings['arduimu'] = 'imu'
replay_mappings['lidar'] = 'lidar'
replay_mappings['flir'] = 'flir'

------------------------------
-- Require the Modules and Utilities
local simple_ipc = require 'simple_ipc'
local mp = require 'msgpack'
local carray = require 'carray'
local cutil = require 'cutil'
local util = require 'util'
local Z = require 'Z'
unix = require 'unix'

------------------------------
-- Read a new log file chunk
-- NOTE: Make sure chunks are small compared to available RAM
local function read_log_file( dtype )
  local log_file_name = log_file_iters[dtype]()
	-- Are we done the logs of this particular type?
  if not log_file_name then
    return false
  end
	-- Open the next file
  local log_f_handle = assert(
	io.open(log_file_name, 'r'),
	string.format( 'Bad read of %s', log_file_name )
	)
  local log_data_chunk = log_f_handle:read( '*a' )
	log_f_handle:close()
	-- TODO: Use the unpacker of msgpack
	local unpacker = msgpack.unpacker(log_data_chunk)
	-- Return the chunk of data
	return unpacker
end

------------------------------
-- Push all data over simple ipc
local function push_simple(self)
	if self.payload then
		self.channel:send( {self.mp_data,self.payload} )
	else
		self.channel:send( self.mp_data )
	end
end

------------------------------
-- TODO: Push meta data over IPC and payload to shm
local function push_shared(self)
end

------------------------------
-- Set the next data element in the log
local function next_datum( self )
	
	-- Return if finished with this log
	if self.finished then
		return false
	end
	
	-- Return if no update is needed
	if not self.needs_update then
		return true
	end

	-- Grab the data from the current chunk
	local data, mp_data = self.chunk:unpack()
	
	-- If there is no more data in this chunk...
	if not data then
		-- Then open the next chunk
		self.chunk = read_log_file( dtype )
		-- If there are no more chunks...
		if not self.chunk then
			-- Then we are finished with this log
			self.finished = true
			-- Allow Garbage Collection on the data and payload
			self.data = nil
			self.mp_data = nil
			self.payload = nil
			return false
		else
			-- Otherwise, grab some data from this new chunk
			data, mp_data = self.chunk:unpack()
			assert(data,'Bad new chunk!')
		end
	end
		
	-- Update the log attributes
	self.data = data
	self.mp_data = mp_data
	self.timestamp = assert( data.t, 'No timestamp given!' )
	
	-- Check if there is a payload, given with sz bytes
	if data.sz then
		-- Open the payload
		--self.payload = 
	end
	
	-- Updated the log
	return true
	
end

------------------------------
-- Get the names of the logs file chunks to use
-- Set up the channels on which to replay these logs
-- Initialize the data chunks and finished states
local logs = {}
for dtype,ch in pairs(replay_mappings) do
	local log_file = LOG_PATH..'/'..dtype..'*'
	local tmp_file_list = assert( 
		io.popen('/bin/ls '..log_file, 'r'), 
		'Bad listing of the log directory!'
	)
	-- Update the global log attributes
	logs[ dtype ].files = tmp_file_list:lines()
	logs[ dtype ].chunk = nil
	logs[ dtype ].needs_update = nil
	logs[ dtype ].finished = false
	logs[ dtype ].channel = simple_ipc.new_publisher( ch )
	logs[ dtype ].timestamp = math.huge
	logs[ dtype ].mp_data = nil
	logs[ dtype ].data = nil
	logs[ dtype ].payload = nil
	logs[ dtype ].push = push_simple
	logs[ dtype ].update = next_datum
end

------------------------------
-- Loop until no logs are left
local still_going = true
while still_going do
	-- Assume we are done unless we receive data
	still_going = false
	
	-- Update each data type
	local min_timestamp = math.huge
	min_log = nil
  for dtype,_ in pairs(replay_mappings) do
		log = logs[ dtype ]
		if log:update() then
			-- We have a valid log with data
			still_going = true
			-- Find the minium timestamp
			if log.timestamp<min_timestamp then
				min_timestamp = log.timestamp
				min_log = log
			end
		end
	end
	assert(min_log,'No minimum timestamp log!')

  -- Sleep depending on the realtime factor
  local t_diff = min_ts - (last_ts or min_ts)
  last_ts = min_ts
    unix.usleep( REALTIME_FACTOR*1e6*t_diff )
  end
	
	-- Push out the logged data
	min_log:push()
	min_log.needs_update = true

end
