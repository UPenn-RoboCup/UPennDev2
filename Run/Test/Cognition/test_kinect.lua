dofile('../../include.lua')
require 'cjpeg'
local simple_ipc = require 'simple_ipc'
local kinect_channel = simple_ipc.setup_publisher('kinect');

require 'Kinect'
local k = Kinect.open()
--local k = Kinect.open('/Users/stephen/Desktop/test_log_kinect2','record')
--local k = Kinect.open('/Users/stephen/Desktop/test_log_kinect2')
print( "Kinect Open!", k )
if not k then
  return
end
-- zoom of 2 to 5 are good.  else not supported
print("Current zoom:",Kinect.zoom(0))
print('Setting zoom to',Kinect.zoom(4))
--for n=1,10 do
counter = 0;
while true do
  counter = counter+1;
  local s = Kinect.update()
  --print( "Kinect Stream: ", s )
  local d = Kinect.retrieve( s )
  --local res = Kinect.zoom( (math.floor(counter/60)%6) +0 );
  --print( "Kinect Data: ", d )
  local z = nil;
  if s==0 then -- depth
    z = cjpeg.compress( d, 320, 240, 1 )
--    print('JDepth:', #z, #z/(320*240))
    kinect_channel:send( {'d', z} );
--    kinect_channel:send( 'a'..z ); -- MATLAB
  elseif s==1 then --color
    z = cjpeg.compress( d, 320, 240 )
    --print('JColor: ', #z, #z/(320*240*3))
    kinect_channel:send( {'c', z} );
  end
end
Kinect.shutdown()
