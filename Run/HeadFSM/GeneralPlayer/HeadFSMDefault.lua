module(..., package.seeall);
local Body = require('Body')
local fsm = require('fsm')
require('gcm')

local headIdle = require('headIdle')
local headStart = require('headStart')
local headReady = require('headReady')
local headReadyLookGoal = require('headReadyLookGoal')
local headScan = require('headScan')
local headTrack = require('headTrack')
local headTrackGoalie = require('headTrackGoalie')
local headKickFollow = require('headKickFollow')
local headLookGoal = require('headLookGoal')
local headSweep = require('headSweep')
local headKick = require('headKick')
local headLog = require('headLog')

sm = fsm.new(headIdle);
sm:add_state(headStart);
sm:add_state(headReady);
sm:add_state(headReadyLookGoal);
sm:add_state(headScan);
sm:add_state(headTrack);
sm:add_state(headTrackGoalie);
sm:add_state(headKickFollow);
sm:add_state(headLookGoal);
sm:add_state(headSweep);
sm:add_state(headKick);
sm:add_state(headLog);



---------------------------------------------
--Game FSM with looking at the goal
---------------------------------------------

sm:set_transition(headStart, 'done', headTrack);

sm:set_transition(headReady, 'done', headReadyLookGoal);

sm:set_transition(headReadyLookGoal, 'timeout', headReady);
sm:set_transition(headReadyLookGoal, 'lost', headReady);

sm:set_transition(headTrack, 'lost', headScan);
sm:set_transition(headTrack, 'timeout', headLookGoal);
sm:set_transition(headTrack, 'sweep', headSweep);

sm:set_transition(headTrackGoalie, 'lost', headScan);

sm:set_transition(headKick, 'ballFar', headTrack);
sm:set_transition(headKick, 'ballLost', headScan);
sm:set_transition(headKick, 'timeout', headTrack);

sm:set_transition(headKickFollow, 'lost', headScan);
sm:set_transition(headKickFollow, 'ball', headTrack);

sm:set_transition(headLookGoal, 'timeout', headTrack);
sm:set_transition(headLookGoal, 'lost', headSweep);

sm:set_transition(headSweep, 'done', headTrack);

sm:set_transition(headScan, 'ball', headTrack);
sm:set_transition(headScan, 'timeout', headScan);

--Transition between player, moving goalie, diving goalie states

sm:set_transition(headTrack, 'goalie', headTrackGoalie);
sm:set_transition(headTrackGoalie, 'player', headTrack);



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
