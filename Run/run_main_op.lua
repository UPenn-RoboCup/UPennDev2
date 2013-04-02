icwd = os.getenv('PWD')
require('init')

require('unix');
--require('main');
require('main_op');

while 1 do 
  tDelay = 0.005*1E6;
--  main.update();
  main_op.update();
  unix.usleep(tDelay);
end

