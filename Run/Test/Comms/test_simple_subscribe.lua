-- Library paths
dofile('../../include.lua')

-- Libraries
local simple_ipc = require 'simple_ipc'
require 'unix'

-- Test setting
local inter_pc = false
local use_poll = true
local use_filter = true

-- Test the filter capability
if use_filter then
  filter = 'ing'
end
print('Using filter {',filter,'}')

-- Set up the subscriber
if inter_pc then
  test_channel = simple_ipc.new_subscriber(5555,filter);
  if filter then
    test_channel2 = simple_ipc.new_subscriber('test','bl');
  end
else
  test_channel = simple_ipc.new_subscriber('test',filter);
  if filter then
    test_channel2 = simple_ipc.new_subscriber('test','bl');
  end
end

if use_poll then
  test_channel.callback = function()
    data, has_more = test_channel:receive()
    print( 'Poller Received', type(data), #data )
    while has_more do
      data, has_more = test_channel:receive()
      print( 'Multipart Received', type(data), #data )
    end
    print()
  end
  if filter then
    test_channel2.callback = function()
      data, has_more = test_channel2:receive()
      print( 'Poller2 Received', type(data), #data, data )
    end
    channel_poll = simple_ipc.wait_on_channels( {test_channel,test_channel2} )
  else
    channel_poll = simple_ipc.wait_on_channels( {test_channel} )
  end
  channel_timeout = 100; -- 100ms timeout
end

-- Begin to receive messages
while true do
  if use_poll then
    npoll = channel_poll:poll(channel_timeout)
    if npoll==0 then
      print('Poller timed out with no messages!')
    else
      print('Received',npoll,'events')
    end
  else
    data, has_more = test_channel:receive()
    print( 'Blocking Received', type(data), #data )
    while has_more do
      data, has_more = test_channel:receive()
      print( 'Multipart Received', type(data), #data )
    end
    print()
  end
end
