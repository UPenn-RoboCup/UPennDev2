require('init')
require('unix')
require('tester')

while 1 do
  tDelay = 0.005*1E6;
  update();
  unix.usleep(tDelay);
end

