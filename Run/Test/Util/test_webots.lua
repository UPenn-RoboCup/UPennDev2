dofile('../../include.lua')

----------------------------------------------------------------------
-- Test Webots Controller
----------------------------------------------------------------------

require('Platform')

print("test_webots: interface to the robot via shared memory...\n");

Platform.entry()
while true do
  Platform.update()
end
Platform.exit()
