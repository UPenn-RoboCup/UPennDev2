dofile('../include.lua')
local simple_ipc = require 'simple_ipc'

inter_pc = false
if inter_pc then
  test_channel = simple_ipc.setup_publisher(5555); --tcp
else
  test_channel = simple_ipc.setup_publisher('test'); --ipc
end
while true do
	test_channel:send( math.random(5) )
  os.execute ('sleep .5')
end
