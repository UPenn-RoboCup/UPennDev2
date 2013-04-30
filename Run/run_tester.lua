cwd = os.getenv('PWD')
local init = require('init')
local unix = require('unix');
local tester = require('tester');

while 1 do 
  tDelay = 0.005*1E6;
  tester.update();
  unix.usleep(tDelay);
end

