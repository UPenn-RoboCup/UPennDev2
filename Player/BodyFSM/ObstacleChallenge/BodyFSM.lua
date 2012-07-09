module(..., package.seeall);

require('fsm')

require('bodyIdle')
require('bodyStop')
require('bodyLearnLUT')
require('bodyDribble')
require('bodyPosition')
require('bodySearch')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStop);
sm:add_state(bodyLearnLUT);
sm:add_state(bodyDribble);
sm:add_state(bodyPosition);
sm:add_state(bodySearch);


-- Obstacle Challenge FSM
sm:set_transition(bodyStart, 'done', bodyPosition);

sm:set_transition(bodyPosition, 'timeout', bodyPosition);
sm:set_transition(bodyPosition, 'ballLost', bodySearch);
sm:set_transition(bodyPosition, 'ballClose', bodyOrbit);
sm:set_transition(bodyPosition, 'done', bodyApproach);
sm:set_transition(bodyPosition, 'dribble', bodyDribble);

sm:set_transition(bodySearch, 'ball', bodyPosition);
sm:set_transition(bodySearch, 'timeout', bodySearch);

sm:set_transition(bodyOrbit, 'timeout', bodyPosition);
sm:set_transition(bodyOrbit, 'ballLost', bodySearch);
sm:set_transition(bodyOrbit, 'ballFar', bodyPosition);
sm:set_transition(bodyOrbit, 'done', bodyApproach);



sm:set_transition(bodyLearnLUT, 'timeout', bodyLearnLUT);
sm:set_transition(bodyLearnLUT, 'done', bodyStart);

sm:set_transition(bodyDribble, 'ballFar', bodyPosition);
sm:set_transition(bodyDribble, 'ballLost', bodySearch);
sm:set_transition(bodyDribble, 'timeout', bodyPosition);
sm:set_transition(bodyDribble, 'done', bodyPosition);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
