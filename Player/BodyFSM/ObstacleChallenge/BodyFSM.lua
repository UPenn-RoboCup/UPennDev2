module(..., package.seeall);

require('fsm')

require('bodyIdle')
require('bodyStop')
require('bodyLearnLUT')
require('bodyDribble')
require('bodyPosition')
require('bodySearch')
require('bodyStart')
require('bodyOrbit')
require('bodyApproach')
require('bodyWait')
require('bodyKick')
require('bodyAvoid')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStop);
sm:add_state(bodyStart);
sm:add_state(bodyLearnLUT);
sm:add_state(bodyDribble);
sm:add_state(bodyPosition);
sm:add_state(bodySearch);
sm:add_state(bodyOrbit);
sm:add_state(bodyApproach);
sm:add_state(bodyWait);
sm:add_state(bodyKick);
sm:add_state(bodyAvoid);


-- Obstacle Challenge FSM
sm:set_transition(bodyWait, 'done', bodyPosition);
sm:set_transition(bodyWait, 'timeout', bodyWait);

sm:set_transition(bodyPosition, 'timeout', bodyPosition);
sm:set_transition(bodyPosition, 'ballLost', bodySearch);
sm:set_transition(bodyPosition, 'ballClose', bodyOrbit);
sm:set_transition(bodyPosition, 'done', bodyApproach);  
--sm:set_transition(bodyPosition, 'done', bodyDribble);  
--sm:set_transition(bodyPosition, 'dribble', bodyDribble);

sm:set_transition(bodySearch, 'ball', bodyPosition);
sm:set_transition(bodySearch, 'timeout', bodySearch);

sm:set_transition(bodyOrbit, 'timeout', bodyPosition);
sm:set_transition(bodyOrbit, 'ballLost', bodySearch);
sm:set_transition(bodyOrbit, 'ballFar', bodyPosition);
sm:set_transition(bodyOrbit, 'done', bodyApproach);

sm:set_transition(bodyApproach, 'ballFar', bodyPosition);
sm:set_transition(bodyApproach, 'ballLost', bodySearch);
sm:set_transition(bodyApproach, 'timeout', bodyPosition);
--sm:set_transition(bodyApproach, 'kick', bodyKick);
--sm:set_transition(bodyApproach, 'dribble', bodyDribble);
sm:set_transition(bodyApproach, 'kick', bodyDribble);
--sm:set_transition(bodyApproach, 'walkkick', bodyWalkKick);

sm:set_transition(bodyLearnLUT, 'timeout', bodyLearnLUT);
sm:set_transition(bodyLearnLUT, 'done', bodyStart);

sm:set_transition(bodyDribble, 'ballFar', bodyPosition);
sm:set_transition(bodyDribble, 'ballLost', bodySearch);
sm:set_transition(bodyDribble, 'timeout', bodyPosition);
--sm:set_transition(bodyDribble, 'timeout', bodyDribble);
sm:set_transition(bodyDribble, 'done', bodyOrbit);

sm:set_transition(bodyPosition, 'fall', bodyPosition);
sm:set_transition(bodyApproach, 'fall', bodyPosition);
sm:set_transition(bodyDribble, 'fall', bodyPosition);


function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
