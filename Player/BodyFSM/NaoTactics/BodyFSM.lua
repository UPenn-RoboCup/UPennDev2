module(..., package.seeall);

require('Body')
require('fsm')
require('gcm')

require('bodyIdle')
require('bodyStart')
require('bodyStop')
require('bodyReady')
require('bodySearch')
require('bodyApproach')
require('bodyKick')
require('bodyOrbit')
require('bodyGotoCenter')
require('bodyAttack')
require('bodyObstacle')
require('bodyObstacleAvoid')
require('bodyWalkKick')

sm = fsm.new(bodyIdle);
sm:add_state(bodyStart);
sm:add_state(bodyStop);
sm:add_state(bodyReady);
sm:add_state(bodySearch);
sm:add_state(bodyApproach);
sm:add_state(bodyKick);
sm:add_state(bodyOrbit);
sm:add_state(bodyGotoCenter);
sm:add_state(bodyAttack);
sm:add_state(bodyObstacle);
sm:add_state(bodyObstacleAvoid);
sm:add_state(bodyWalkKick);

sm:set_transition(bodyStart, 'done', bodyAttack);

sm:set_transition(bodyAttack, 'timeout', bodyAttack);
sm:set_transition(bodyAttack, 'ballLost', bodySearch);
sm:set_transition(bodyAttack, 'ballAlign', bodyOrbit);
sm:set_transition(bodyAttack, 'approach', bodyApproach);
sm:set_transition(bodyAttack, 'obstacle', bodyObstacle);

sm:set_transition(bodyObstacle, 'clear', bodyAttack);
sm:set_transition(bodyObstacle, 'timeout', bodyObstacleAvoid);

sm:set_transition(bodyObstacleAvoid, 'clear', bodyAttack);
sm:set_transition(bodyObstacleAvoid, 'timeout', bodyAttack);

sm:set_transition(bodySearch, 'ball', bodyAttack);
sm:set_transition(bodySearch, 'timeout', bodyGotoCenter);

sm:set_transition(bodyGotoCenter, 'ballFound', bodyAttack);
sm:set_transition(bodyGotoCenter, 'done', bodySearch);
sm:set_transition(bodyGotoCenter, 'timeout', bodySearch);

sm:set_transition(bodyOrbit, 'timeout', bodyAttack);
sm:set_transition(bodyOrbit, 'ballLost', bodySearch);
sm:set_transition(bodyOrbit, 'ballFar', bodyAttack);
sm:set_transition(bodyOrbit, 'done', bodyApproach);

sm:set_transition(bodyApproach, 'ballFar', bodyAttack);
sm:set_transition(bodyApproach, 'ballAlign', bodyOrbit);
sm:set_transition(bodyApproach, 'ballLost', bodySearch);
sm:set_transition(bodyApproach, 'timeout', bodyAttack);
sm:set_transition(bodyApproach, 'kick', bodyKick);
sm:set_transition(bodyApproach, 'walkKick', bodyWalkKick);

sm:set_transition(bodyKick, 'timeout', bodyAttack);
sm:set_transition(bodyKick, 'done', bodyAttack);

sm:set_transition(bodyWalkKick, 'timeout', bodyAttack);
sm:set_transition(bodyWalkKick, 'done', bodyAttack);

sm:set_transition(bodyAttack, 'fall', bodyAttack);
sm:set_transition(bodyApproach, 'fall', bodyAttack);
sm:set_transition(bodyKick, 'fall', bodyAttack);
sm:set_transition(bodyWalkKick, 'fall', bodyAttack);

-- set state debug handle to shared memory settor
sm:set_state_debug_handle(gcm.set_fsm_body_state);


function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
