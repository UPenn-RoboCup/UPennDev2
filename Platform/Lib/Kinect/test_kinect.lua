local kinect = require 'kinect'
local k = Kinect.open()
--s = Kinect.open('/tmp/nifile')
--s = Kinect.open('/tmp/nifile','record')
print( "Kinect Status: ", k )
if not k then
  return
end
print( "Current Zoom Level: ", Kinect.zoom(0) )
print( "Setting Zoom Level: ", Kinect.zoom(4) )

for n=1,10 do
  local s = Kinect.update()
  print( "Kinect Stream: ", s )
  local d = Kinect.retrieve( s )
  print( "Kinect Data: ", d )
end
Kinect.shutdown()
