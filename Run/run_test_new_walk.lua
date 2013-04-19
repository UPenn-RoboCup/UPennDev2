cwd = os.getenv('PWD')
local init = require('init')
local unix = require('unix');
local test_new_walk = require('test_new_walk');

while 1 do 
  test_new_walk.update();
  unix.usleep(100);
end

