module(..., package.seeall);
local Body = require('Body')
local fsm = require('fsm')
local gcm = require('gcm')

local headIdle = require('headIdle')
local headStart = require('headStart')
local headReady = require('headReady')
local headScan = require('headScan')
local headTrack = require('headTrack')
local headSweep = require('headSweep')

sm = fsm.new(headIdle);
sm:add_state(headStart);
sm:add_state(headReady);
sm:add_state(headScan);
sm:add_state(headTrack);
sm:add_state(headSweep);

sm:set_transition(headStart, 'done', headTrack);

sm:set_transition(headReady, 'done', headReady);

sm:set_transition(headTrack, 'lost', headScan);
sm:set_transition(headTrack, 'timeout', headScan);

sm:set_transition(headSweep, 'done', headTrack);

sm:set_transition(headScan, 'ball', headTrack);
sm:set_transition(headScan, 'timeout', headSweep);

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
