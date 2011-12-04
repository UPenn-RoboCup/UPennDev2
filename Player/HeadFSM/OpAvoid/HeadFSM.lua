module(..., package.seeall);

require('Body')
require('fsm')
require('gcm')

require('headIdle')
require('headScan')
require('headTrack')
require('headSweep')

sm = fsm.new(headIdle);
sm:add_state(headScan);
sm:add_state(headTrack);
sm:add_state(headSweep);

sm:set_transition(headIdle, "timeout", headTrack);
sm:set_transition(headTrack, "lost", headScan);
sm:set_transition(headTrack, "timeout", headTrack);
sm:set_transition(headScan, "ball", headTrack);
sm:set_transition(headScan, "timeout", headScan);

sm:set_transition(headSweep, "timeout", headSweep);

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
