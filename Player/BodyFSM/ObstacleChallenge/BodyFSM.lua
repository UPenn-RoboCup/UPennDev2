module(..., package.seeall);

require('fsm')

require('bodyIdle')
require('bodyLearnLUT')

sm = fsm.new(bodyIdle);
sm:add_state(bodyLearnLUT);


-- Obstacle Challenge FSM

sm:set_transition(bodyIdle, 'timeout', bodyIdle);

sm:set_transition(bodyLearnLUT, 'timeout', bodyLearnLUT);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
