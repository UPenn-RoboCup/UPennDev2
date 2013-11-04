local simple_ipc = require 'simple_ipc'
local test_channel = simple_ipc.new_subscriber('test');
while true do
	local x = test_channel:receive()
	print('Got',#x)
end
