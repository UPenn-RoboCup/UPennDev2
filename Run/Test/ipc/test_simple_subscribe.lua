local ffi = require 'ffi'
local simple_ipc = require 'simple_ipc'
--local test_channel = simple_ipc.setup_subscriber('test');
local test_channel = simple_ipc.setup_subscriber(5555);
local buffer_ct = ffi.new('double[1]',0);
while true do
	test_channel:receive( buffer_ct, 1 )
	print(buffer_ct[0])
end