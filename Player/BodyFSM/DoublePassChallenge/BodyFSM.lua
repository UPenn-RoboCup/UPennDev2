module(..., package.seeall);

require('fsm')
require('bodyIdle')
require('bodyStart')
require('bodyStop')
require('bodyReady')

require('bodySearch')
require('bodyGetPosition')
require('bodyChase')
require('bodyApproach')
require('bodyPass')
require('bodyTask')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyStop);
sm:add_state(bodyReady);

sm:add_state(bodySearch);
sm:add_state(bodyGetPosition);
sm:add_state(bodyChase);
sm:add_state(bodyApproach);
sm:add_state(bodyPass);
sm:add_state(bodyTask);

sm:set_transition(bodyStart, 'done', bodyTask);

--Main state that dispatches each passes
sm:set_transition(bodyTask, 'move', bodyGetPosition);
sm:set_transition(bodyTask, 'pass', bodyChase);

--Get into position and wait for pass
sm:set_transition(bodyGetPosition, 'done', bodyTask);

-- Chase after the ball
sm:set_transition(bodyChase, 'ballLost', bodySearch);
sm:set_transition(bodyChase, 'ballClose', bodyApproach);
sm:set_transition(bodyChase, 'timeout', bodyChase);

-- Approach the ball 
sm:set_transition(bodyApproach, 'ballFar', bodyChase);
sm:set_transition(bodyApproach, 'ballLost', bodySearch);
sm:set_transition(bodyApproach, 'kick', bodyPass);
sm:set_transition(bodyApproach, 'timeout', bodyChase);

-- Search for the ball
sm:set_transition(bodySearch, 'ball', bodyChase);
sm:set_transition(bodySearch, 'timeout', bodySearch);

-- Pass the ball
sm:set_transition(bodyPass, 'timeout', bodyTask);
sm:set_transition(bodyPass, 'done', bodyTask);

-- If you fall, what do you do?
sm:set_transition(bodyGetPosition, 'fall', bodyGetPosition);
sm:set_transition(bodyChase, 'fall', bodySearch);
sm:set_transition(bodyApproach, 'fall', bodySearch);
sm:set_transition(bodyPass, 'fall', bodySearch);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
