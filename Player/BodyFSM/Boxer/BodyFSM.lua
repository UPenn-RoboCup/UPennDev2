module(..., package.seeall);

require('fsm')
require('bodyIdle')
require('bodySearch')

sm = fsm.new(bodyIdle);
sm:add_state(bodySearch);

-- Search for the stretcher
sm:set_transition(bodySearch, 'timeout', bodySearch);

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
