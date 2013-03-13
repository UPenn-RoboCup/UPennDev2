dofile('../../include.lua')
local simple_ipc = require 'simple_ipc'

inter_pc = true
if inter_pc then
  test_channel = simple_ipc.setup_publisher(5555); --tcp
else
  test_channel = simple_ipc.setup_publisher('test'); --ipc
end
for i=1,10 do
  print('Sending payload:',i)
	test_channel:send( i )
  os.execute ('sleep .5')
end
