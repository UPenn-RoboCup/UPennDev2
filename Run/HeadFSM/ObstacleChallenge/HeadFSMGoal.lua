module(..., package.seeall);
local fsm = require('fsm')

local headIdle = require('headIdle')
local headLearnLUT = require('headLearnLUT')
local headStart = require('headStart')
local headTrack = require('headTrack')
local headSweep = require('headSweep')
local headLookGoal = require('headLookGoal')
local headScan = require('headScan')
local headKick = require('headKick')
local headScanObs = require('headScanObs')
local headBack = require('headBack')


sm = fsm.new(headIdle);
sm:add_state(headStart);
sm:add_state(headLearnLUT);
sm:add_state(headTrack)
sm:add_state(headSweep)
sm:add_state(headLookGoal)
sm:add_state(headScan)
sm:add_state(headScanObs)
sm:add_state(headKick)
sm:add_state(headBack)

sm:set_transition(headLearnLUT, 'timeout', headScanObs);
sm:set_transition(headLearnLUT, 'done', headLookGoal);

sm:set_transition(headLookGoal, 'timeout', headBack);
sm:set_transition(headLookGoal, 'lost', headSweep);

sm:set_transition(headBack, 'timeout', headScanObs);

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
