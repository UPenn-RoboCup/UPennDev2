module(..., package.seeall);
require('fsm')

require('headIdle')
require('headLearnLUT')
require('headTrack')
require('headSweep')
require('headLookGoal')
require('headScan')
require('headScanObs')
require('headKick')
require('headViewBlocked')


sm = fsm.new(headIdle)
sm:add_state(headLearnLUT)
sm:add_state(headTrack)
sm:add_state(headSweep)
sm:add_state(headLookGoal)
sm:add_state(headScan)
sm:add_state(headKick)
sm:add_state(headScanObs)
sm:add_state(headViewBlocked)

sm:set_transition(headLearnLUT, 'timeout', headScan);
sm:set_transition(headLearnLUT, 'done', headLookGoal);

sm:set_transition(headTrack, 'lost', headScan);
sm:set_transition(headTrack, 'timeout', headScanObs);
sm:set_transition(headTrack, 'sweep', headSweep);
sm:set_transition(headTrack, 'blocked', headViewBlocked);

sm:set_transition(headScanObs, 'timeout', headLookGoal);
sm:set_transition(headScanObs, 'blocked', headViewBlocked);

sm:set_transition(headLookGoal, 'timeout', headTrack);
sm:set_transition(headLookGoal, 'lost', headSweep);
sm:set_transition(headLookGoal, 'blocked', headViewBlocked);

sm:set_transition(headSweep, 'done', headTrack);

sm:set_transition(headScan, 'ball', headTrack);
sm:set_transition(headScan, 'timeout', headScan);
sm:set_transition(headScan, 'blocked', headViewBlocked);

sm:set_transition(headViewBlocked, 'timeout', headScan);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
