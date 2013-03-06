package.path = "./../../Util/?.lua;"..package.path;
local simple_ipc = require 'simple_ipc'
--local test_channel = simple_ipc.setup_publisher('test'); --ipc
local test_channel = simple_ipc.setup_publisher(5555); --tcp
os.execute ('sleep .5')
i = 0
while true do
  i = i+1
	test_channel:send( i )
  os.execute ('sleep .5')
end
