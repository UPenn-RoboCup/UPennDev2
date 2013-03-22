dofile('../../include.lua')
local simple_ipc = require 'simple_ipc'
require 'unix'
local kinect_channel = simple_ipc.setup_publisher('img');
--local kinect_channel = simple_ipc.setup_subscriber('img');

while true do
--for n=1,10 do
  kinect_channel:send( 'a hello' );
  unix.usleep(1e6);
  --local val, more = kinect_channel:receive();
  --print('Receive',val,more) 
end
-- TODO: how to shutdown the zeromq ipc?
