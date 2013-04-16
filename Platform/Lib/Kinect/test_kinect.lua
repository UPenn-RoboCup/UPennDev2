-- Kinect Lua test script (c) Stephen McGill 2013
local kinect = require 'kinect'
local k = kinect.open()
--s = kinect.open('/tmp/nifile')
--s = kinect.open('/tmp/nifile','record')
print( "Kinect Status: ", k )
if not k then
  return
end
print( "Current Zoom Level: ", kinect.zoom(0) )
print( "Setting Zoom Level: ", kinect.zoom(4) )

for n=1,10 do
  local s = kinect.update()
  print( "Kinect Stream: ", s )
  local d = kinect.retrieve( s )
  print( "Kinect Data: ", d )
end
kinect.shutdown()
