module(..., package.seeall);

require 'fsm'
require 'side'
require 'fore'
require 'up'
require 'boxercm'

sm = fsm.new(side); -- right hand following.  Let's see how duplication is
sm:add_state(fore);
sm:add_state(up);

sm:set_transition(side,'forward',fore)
sm:set_transition(side,'up',up)

sm:set_transition(fore,'up',up)
sm:set_transition(fore,'down',side)

sm:set_transition(up,'down',side)

-- For debugging
sm:set_state_debug_handle(boxercm.set_fsm_state);
boxercm.set_fsm_state('side')

function entry()
  sm:entry()
end

function update()
  sm:update();
end

function exit()
  sm:exit();
end

function event(e)
  sm:add_event(e);
end
