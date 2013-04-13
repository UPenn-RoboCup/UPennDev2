-- Library paths
dofile('../../include.lua')

-- Libraries
local simple_ipc = require 'simple_ipc'
require 'unix'

-- Test setting
local inter_pc = false
local use_multipart = true;
local use_filter = true;

-- Test the filter capability
if use_filter then
  filter = 'ing'
end
print('Using filter {',filter,'}')

-- Set up the subscriber
if inter_pc then
  test_channel = simple_ipc.new_publisher(5555,filter);
  if filter then
    test_channel2 = simple_ipc.new_publisher(5555,'bl');
  end
else
  test_channel = simple_ipc.new_publisher('test',filter);
  if filter then
    test_channel2 = simple_ipc.new_publisher('test','bl');
  end
end

-- Begin to receive messages
local ret = false;
while true do
  if use_multipart then
    ret = test_channel:send( {'hello','world'} )
  else
    ret = test_channel:send('hello')
  end
  if filter then
    ret = test_channel2:send('thorn in my side')
  end
  if not ret then
    print("Failed to send message!")
  else
    print('Sent message.')
  end
  -- Send a message once per second
  unix.usleep(1e6)
end
