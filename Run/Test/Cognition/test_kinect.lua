dofile('../../include.lua')
require 'cjpeg'
local simple_ipc = require 'simple_ipc'
local kinect_channel = simple_ipc.setup_publisher('kinect');

require 'Kinect'
print( "Opening the Kinect..." )
Kinect.open()
print( "Kinect Open!")

--for n=1,10 do
while true do
  local s = Kinect.update()
  --print( "Kinect Stream: ", s )
  local d = Kinect.retrieve( s )
  --print( "Kinect Data: ", d )
  local z = nil;
  if s==0 then -- depth
    z = cjpeg.compress( d, 320, 240, 1 )
    --print('Depth compression ratio: ', #z/(320*240))
    kinect_channel:send( z );
  elseif s==1 then --color
    z = cjpeg.compress( d, 320, 240 )
    --print('Color compression ratio: ', #z/(320*240*3))
    kinect_channel:send( z );
  end
end
Kinect.shutdown()

-- TODO: how to shutdown the zeromq ipc?
