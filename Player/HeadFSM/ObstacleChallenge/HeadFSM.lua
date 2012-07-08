module(..., package.seeall);
require('fsm')

require('headIdle')
require('headLearnLUT')

sm = fsm.new(headIdle);
sm:add_state(headLearnLUT);

sm:set_transition(headLearnLUT, 'timeout', headLearnLUT);

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end
