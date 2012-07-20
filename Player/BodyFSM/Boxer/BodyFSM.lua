module(..., package.seeall);

require('fsm')
require('bodyIdle')
require('bodyBox')

sm = fsm.new(bodyIdle);
sm:add_state(bodyBox);

-- Set transitions
sm:set_transition(bodyIdle, 'bell', bodyBox); -- Bell starts the match

-- Always box, unless lost data connection / skeleton
sm:set_transition(bodyBox, 'lost', bodyIdle);
sm:set_transition(bodyBox, 'timeout', bodyBox);

-- If you fall, what do you do?
--sm:set_transition(bodyChase, 'fall', bodySearch);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
