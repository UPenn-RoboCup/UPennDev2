module(..., package.seeall);
require('Body')
require('fsm')
require('gcm')

require('headIdle')
require('headStart')
require('headReady')
require('headReadyLookGoal')
require('headScan')
require('headTrack')
require('headKick')
require('headKickFollow')
require('headLookGoal')
require('headLog')
require('headSweep')

sm = fsm.new(headIdle);
sm:add_state(headStart);
sm:add_state(headReady);
sm:add_state(headReadyLookGoal);
sm:add_state(headScan);
sm:add_state(headTrack);
sm:add_state(headKick);
sm:add_state(headLog);
sm:add_state(headKickFollow);
sm:add_state(headLookGoal);
sm:add_state(headSweep);

sm:set_transition(headStart, 'done', headIdle);

sm:set_transition(headReady, 'done', headIdle);

sm:set_transition(headTrack, 'lost', headIdle);
sm:set_transition(headTrack, 'timeout', headIdle);

sm:set_transition(headKick, 'ballFar', headIdle);
sm:set_transition(headKick, 'ballLost', headIdle);
sm:set_transition(headKick, 'timeout', headIdle);

sm:set_transition(headKickFollow, 'lost', headIdle);
sm:set_transition(headKickFollow, 'ball', headIdle);

sm:set_transition(headScan, 'ball', headIdle);
sm:set_transition(headScan, 'timeout', headIdle);

-- set state debug handle to shared memory settor
sm:set_state_debug_handle(gcm.set_fsm_head_state);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
