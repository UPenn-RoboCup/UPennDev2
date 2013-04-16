local skeleton = require 'skeleton'
local s = skeleton.open()
print( "Kinect Status: ", s )

for n=1,10 do
  local rc = skeleton.update()
end
skeleton.shutdown()
