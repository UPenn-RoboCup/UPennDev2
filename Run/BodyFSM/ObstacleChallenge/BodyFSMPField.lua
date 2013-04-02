module(..., package.seeall);

require('fsm')

require('bodyIdle')
require('bodyLearnLUT')
require('bodyPosition')
require('bodyWait')
require('bodyStop')
require('bodyApproachGoal')
require('bodyAvoid')

sm = fsm.new(bodyIdle);
sm:add_state(bodyPosition);
sm:add_state(bodyWait);
sm:add_state(bodyStop);
sm:add_state(bodyAvoid);
sm:add_state(bodyApproachGoal);


-- Obstacle Challenge FSM
sm:set_transition(bodyWait, 'done', bodyApproachGoal);
sm:set_transition(bodyWait, 'timeout', bodyWait);

sm:set_transition(bodyApproachGoal, 'timeout', bodyApproachGoal)
sm:set_transition(bodyApproachGoal, 'obstacle', bodyAvoid)
sm:set_transition(bodyApproachGoal, 'done', bodyStop)

sm:set_transition(bodyAvoid, 'done', bodyApproachGoal)
sm:set_transition(bodyAvoid, 'timeout', bodyApproachGoal)
sm:set_transition(bodyAvoid, 'obstacle', bodyAvoid)

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
