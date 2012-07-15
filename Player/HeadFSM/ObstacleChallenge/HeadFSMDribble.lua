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


sm = fsm.new(headIdle);
sm:add_state(headStart);
sm:add_state(headLearnLUT);
sm:add_state(headTrack)
sm:add_state(headSweep)
sm:add_state(headLookGoal)
sm:add_state(headScan)
sm:add_state(headKick)

sm:set_transition(headLearnLUT, 'timeout', headLearnLUT);
sm:set_transition(headLearnLUT, 'done', headScan);

sm:set_transition(headStart, 'done', headTrack);

sm:set_transition(headTrack, 'lost', headScan);
sm:set_transition(headTrack, 'timeout', headLookGoal);
sm:set_transition(headTrack, 'sweep', headSweep);

sm:set_transition(headLookGoal, 'timeout', headTrack);
sm:set_transition(headLookGoal, 'lost', headSweep);

sm:set_transition(headSweep, 'done', headTrack);

sm:set_transition(headScan, 'ball', headTrack);
sm:set_transition(headScan, 'timeout', headScan);

sm:set_transition(headKick, 'ballFar', headTrack);
sm:set_transition(headKick, 'ballLost', headScan);
sm:set_transition(headKick, 'timeout', headTrack);




function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
