module(..., package.seeall);

require 'relax'
require 'fore'
require 'up'
require 'boxercm'

sm = fsm.new(relax); -- right hand following.  Let's see how duplication is
sm:add_state(fore);
sm:add_state(up);

sm:set_transition(relax,'forward',fore)
sm:set_transition(relax,'up',up)

sm:set_transition(fore,'up',up)
sm:set_transition(fore,'down',relax)

sm:set_transition(up,'down',relax)

-- For debugging
sm:set_state_debug_handle(boxercm.set_fsm_state);


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
