module(..., package.seeall);
local Body = require('Body')
local fsm = require('fsm')
local gcm = require('gcm')

local headIdle = require('headIdle')
local headStart = require('headStart')
local headReady = require('headReady')

sm = fsm.new(headIdle);
sm:add_state(headStart);
sm:add_state(headReady);

sm:set_transition(headStart, 'done', headIdle);
sm:set_transition(headReady, 'done', headIdle);

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
