module(..., package.seeall);
require('fsm')

require('headIdle')
require('headLearnLUT')
require('headStart')
require('headTrack')
require('headSweep')
require('headLookGoal')
require('headScan')
require('headKick')
require('headScanObs')


sm = fsm.new(headIdle);
sm:add_state(headStart);
sm:add_state(headLearnLUT);
sm:add_state(headTrack)
sm:add_state(headSweep)
sm:add_state(headLookGoal)
sm:add_state(headScan)
sm:add_state(headScanObs)
sm:add_state(headKick)

sm:set_transition(headLearnLUT, 'timeout', headScanObs);
sm:set_transition(headLearnLUT, 'done', headLookGoal);

sm:set_transition(headLookGoal, 'timeout', headScanObs);
sm:set_transition(headLookGoal, 'lost', headSweep);

sm:set_transition(headSweep, 'done', headScanObs);

--sm:set_transition(headScanObs, 'timeout', headScanObs);
sm:set_transition(headScanObs, 'timeout', headLookGoal);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
