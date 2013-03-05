require 'unix'
local simple_ipc = require 'simple_ipc'
--local test_channel = simple_ipc.setup_publisher('test'); --ipc
local test_channel = simple_ipc.setup_publisher(5555); --tcp
while true do
	test_channel:send( unix.time() )
	unix.usleep(1e6)
end