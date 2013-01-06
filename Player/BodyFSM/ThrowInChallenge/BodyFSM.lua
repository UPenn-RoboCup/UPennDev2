module(..., package.seeall);

require('fsm')
require('bodyIdle')
require('bodyStart')
require('bodyStop')
require('bodyReady')

require('bodySearch')
require('bodyChase')
require('bodyApproachGrip')
require('bodyPickup')
require('bodyAim')
require('bodyThrow')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyStop);
sm:add_state(bodyReady);

sm:add_state(bodySearch);
sm:add_state(bodyChase);
sm:add_state(bodyApproachGrip);
sm:add_state(bodyPickup);
sm:add_state(bodyAim);
sm:add_state(bodyThrow);

sm:set_transition(bodyStart, 'done', bodySearch);

-- Search for the ball
sm:set_transition(bodySearch, 'ball', bodyChase);
sm:set_transition(bodySearch, 'timeout', bodySearch);

-- Chase after the ball
sm:set_transition(bodyChase, 'ballLost', bodySearch);
sm:set_transition(bodyChase, 'ballClose', bodyApproachGrip);
sm:set_transition(bodyChase, 'timeout', bodyChase);

-- Approach the ball (and into position)
sm:set_transition(bodyApproachGrip, 'ballFar', bodyChase);
sm:set_transition(bodyApproachGrip, 'ballLost', bodySearch);
sm:set_transition(bodyApproachGrip, 'pickup', bodyPickup);
sm:set_transition(bodyApproachGrip, 'timeout', bodyChase);

-- Pickup the ball
sm:set_transition(bodyPickup, 'timeout', bodyApproachGrip);
sm:set_transition(bodyPickup, 'done', bodyAim);

-- Aim the throw
sm:set_transition(bodyAim, 'done', bodyThrow);
sm:set_transition(bodyAim, 'timeout', bodyThrow);

-- Throw the ball
sm:set_transition(bodyThrow, 'done', bodySearch);

-- If you fall, what do you do?
sm:set_transition(bodyChase, 'fall', bodySearch);
sm:set_transition(bodyApproachGrip, 'fall', bodySearch);
sm:set_transition(bodyPickup, 'fall', bodyIdle);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
