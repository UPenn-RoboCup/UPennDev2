require 'Kinect'
--s = Kinect.open()
s = Kinect.open('/tmp/nifile')
--s = Kinect.open('/tmp/nifile','record')
print( "Kinect Status: ", s )

for n=1,10 do
  s = Kinect.update()
  print( "Kinect Stream: ", s )
  d = Kinect.retrieve( s )
  print( "Kinect Data: ", d )
end
Kinect.shutdown()
