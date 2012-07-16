module(..., package.seeall);

require('fsm')

require('bodyIdle')
require('bodyStop')
require('bodyDribble')
require('bodyPosition')
require('bodySearch')
require('bodyApproach')
require('bodyWait')
require('bodyAvoid')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStop);
sm:add_state(bodyDribble);
sm:add_state(bodyPosition);
sm:add_state(bodySearch);
sm:add_state(bodyApproach);
sm:add_state(bodyWait);
sm:add_state(bodyAvoid);


-- Obstacle Challenge FSM
sm:set_transition(bodyWait, 'done', bodySearch);
sm:set_transition(bodyWait, 'timeout', bodyWait);

sm:set_transition(bodyPosition, 'timeout', bodyPosition);
sm:set_transition(bodyPosition, 'ballLost', bodySearch);
sm:set_transition(bodyPosition, 'ballClose', bodyApproach);
sm:set_transition(bodyPosition, 'done', bodyApproach);  
sm:set_transition(bodyPosition, 'obstacle', bodyAvoid);
sm:set_transition(bodyPosition, 'done', bodyStop);

sm:set_transition(bodySearch, 'ball', bodyPosition);
sm:set_transition(bodySearch, 'timeout', bodySearch);

sm:set_transition(bodyApproach, 'ballFar', bodyPosition);
sm:set_transition(bodyApproach, 'ballLost', bodySearch);
sm:set_transition(bodyApproach, 'timeout', bodyPosition);
sm:set_transition(bodyApproach, 'kick', bodyDribble);
sm:set_transition(bodyApproach, 'obstacle', bodyAvoid);
sm:set_transition(bodyApproach, 'done', bodyStop);

sm:set_transition(bodyDribble, 'ballFar', bodyPosition);
sm:set_transition(bodyDribble, 'ballLost', bodySearch);
sm:set_transition(bodyDribble, 'timeout', bodyPosition);
sm:set_transition(bodyDribble, 'done', bodyStop);
sm:set_transition(bodyDribble, 'obstacle', bodyAvoid);

sm:set_transition(bodyAvoid, 'obstacle', bodyAvoid)
sm:set_transition(bodyAvoid, 'done', bodyPosition)
sm:set_transition(bodyAvoid, 'timeout', bodyPosition)

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
