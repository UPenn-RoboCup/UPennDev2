cwd = os.getenv('PWD')
local init = require('init')

local unix = require('unix');
local main = require('main');

while 1 do 
  tDelay = 0.005*1E6;
  main.update();
  unix.usleep(tDelay);
end

