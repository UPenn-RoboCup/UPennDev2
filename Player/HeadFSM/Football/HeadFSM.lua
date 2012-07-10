module(..., package.seeall);
require('Body')
require('fsm')
require('gcm')

require('headIdle')
require('headStart')
require('headReady')

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
