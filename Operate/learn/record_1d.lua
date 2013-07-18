dofile'../include.lua'

-- Set up the ZMQ listening
local simple_ipc = require'simple_ipc'
local skeleton_ch1 = simple_ipc.new_publisher'skeleton1'
local skeleton_ch2 = simple_ipc.new_publisher'skeleton2'

local function update()
	
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

-- Begin listening loop
local wait_channels = {}
table.insert(wait_channels,skeleton_ch1)
table.insert(wait_channels,skeleton_ch2)
local channel_poll = simple_ipc.wait_on_channels( wait_channels )
channel_poll:start()