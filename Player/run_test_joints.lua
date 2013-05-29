require('test_joints')
require('unix')
require('os')

while true do
  os.execute("clear")
  tDelay = 0.52;
  test_joints.update();
  unix.sleep(tDelay);
end
