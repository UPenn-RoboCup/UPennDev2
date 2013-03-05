dofile('../include.lua')
local ffi = require 'ffi'
local simple_ipc = require 'simple_ipc'
inter_pc = false
if inter_pc then
  test_channel = simple_ipc.setup_subscriber(5555);
else
  test_channel = simple_ipc.setup_subscriber('test');
end
local buffer_ct = ffi.new('double[1]',0);
while true do
	test_channel:receive( buffer_ct, 1 )
	print(buffer_ct[0])
end
