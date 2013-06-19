module(..., package.seeall);

require('fsm')

require('lidarPan')
require('lidarIdle')


sm = fsm.new(lidarPan);
sm:add_state(lidarIdle);

sm:set_transition(lidarPan, 'stop', lidarIdle);
sm:set_transition(lidarIdle, 'start', lidarPan);



function entry()
  sm:entry();
end

function update()
  sm:update();
end

function exit()
  sm:exit()
end
