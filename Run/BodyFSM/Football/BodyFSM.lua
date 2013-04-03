module(..., package.seeall);

require('fsm')
require('bodyIdle')
require('bodyChase')
--require('bodyApproach')
require('bodyStartingPos');
require('bodyWait');

sm = fsm.new(bodyIdle);
sm:add_state(bodyChase);
--sm:add_state(bodySearch); -- Real robot only
sm:add_state(bodyWait);
sm:add_state(bodyStartingPos);

-- Search for the enemy (only on the real robot...)
--sm:set_transition(bodySearch, 'stretcher', bodyChase);
--sm:set_transition(bodySearch, 'timeout', bodySearch);

-- Chase after each other
sm:set_transition(bodyChase, 'close', bodyStartingPos);
sm:set_transition(bodyChase, 'timeout', bodyChase);

-- Put both players back to the starting point
sm:set_transition(bodyStartingPos, 'timeout', bodyStartingPos);
sm:set_transition(bodyStartingPos, 'done', bodyWait);

-- Done waiting, then start the next trial
sm:set_transition(bodyWait, 'done', bodyChase);

-- If you fall, what do you do?
sm:set_transition(bodyChase, 'fall', bodyChase);
sm:set_transition(bodyStartingPos, 'fall', bodyStartingPos);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
