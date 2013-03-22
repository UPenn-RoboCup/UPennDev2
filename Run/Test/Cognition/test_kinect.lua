dofile('../../include.lua')
require 'cjpeg'
local simple_ipc = require 'simple_ipc'
local kinect_channel = simple_ipc.setup_publisher('kinect');

require 'Kinect'
print( "Opening the Kinect..." )
--k = Kinect.open()
k = Kinect.open('/Users/stephen/Desktop/test_log_kinect')
print( "Kinect Open!", k )

--for n=1,10 do
while true do
  local s = Kinect.update()
--  print( "Kinect Stream: ", s )
  local d = Kinect.retrieve( s )
--  print( "Kinect Data: ", d )
  local z = nil;
  if s==1 then -- depth
    z = cjpeg.compress( d, 320, 240, 1 )
    print('Depth:', #z, #z/(320*240))
    kinect_channel:send( {'d', z} );
  elseif s==0 then --color
    z = cjpeg.compress( d, 320, 240 )
    print('Color: ', #z, #z/(320*240*3))
    kinect_channel:send( {'c', z} );
  end
end
Kinect.shutdown()

-- TODO: how to shutdown the zeromq ipc?
