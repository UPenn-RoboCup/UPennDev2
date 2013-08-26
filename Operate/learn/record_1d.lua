dofile'../../include.lua'
require'unix'

-- Configuration
-- 10 seconds per recording
local keyframe_time_length = 10
local nCountdown = 5

-- Set up the ZMQ listening
local simple_ipc = require'simple_ipc'
local skeleton_ch1 = simple_ipc.new_publisher'skeleton1'
local skeleton_ch2 = simple_ipc.new_publisher'skeleton2'
local wait_channels = {}
table.insert(wait_channels,skeleton_ch1)
table.insert(wait_channels,skeleton_ch2)
local channel_poll = simple_ipc.wait_on_channels( wait_channels )

-- FSM for the recording
local motion_count = 0
local keyframe_time = unix.time()
local keyframe_time0 = unix.time()
local function entry()
	-- Clear some space
	for i=1,10 do
		io.write('\n')
	end
	io.flush()
			
	-- Begin the countdown
	for i=nCountdown,1,-1 do
		io.write(i..'...')
		io.flush()
		unix.sleep(1)
	end
	io.write(0)
	io.flush()
	
	-- Ensure that we are still listening
	local channel_timeout = 1e3
	local npoll = channel_poll:poll(channel_timeout)
	assert(npoll>0,'No skeletons produced! '..npoll)
	
	-- Start polling for keyframes
	channel_poll:start()
	
end
local function exit()
	-- Stop the poller
	channel_poll:stop()
	motion_count = motion_count + 1
	
	-- Save the keyframe
	print('Saved motion',motion_count)
end
local function update()
	if keyframe_time-keyframe_time0>keyframe_time_length then
		exit()
		-- Begin the next keyframe
		entry()
	end
end

-- Setup the callbacks
skeleton_ch1.callback = function()
	local meta, has_more = skeleton_ch1:receive()
	assert(has_more,'Head | No payload')
	local skeleton_str = skeleton_ch1:receive()
	local metadata = mp.unpack(meta)
	local skeleton = mp.unpack(skeleton_str)
	update()
end
skeleton_ch2.callback = function()
	local meta, has_more = skeleton_ch2:receive()
	assert(has_more,'Head | No payload')
	local skeleton_str = skeleton_ch2:receive()
	local metadata = mp.unpack(meta)
	local skeleton = mp.unpack(skeleton_str)
	update(metadata,skeleton)
end

-- Start the countdown...
entry()