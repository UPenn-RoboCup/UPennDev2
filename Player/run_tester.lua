require ('tester')
require('unix')
require('main')

while 1 do
  tDelay = 0.005*1E6;
  tester.update();
  unix.usleep(tDelay);
end
