dofile('../include.lua')

----------------------------------------------------------------------
-- Test Webots Controller
----------------------------------------------------------------------

require('Body')

print("test_webots: interface to the robot via shared memory...\n");

Body.entry()
while true do
  Body.update()
end
Body.exit()
