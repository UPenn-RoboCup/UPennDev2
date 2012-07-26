module(..., package.seeall);

require 'fsm'
require 'util'
require 'side'
require 'fore'
require 'up'
require 'libboxer'

smR = fsm.new(side); -- right hand following.  Let's see how duplication is
smR:add_state(fore);
smR:add_state(up);

smR:set_transition(side,'forward',fore)
smR:set_transition(side,'up',up)

smR:set_transition(fore,'up',up)
smR:set_transition(fore,'down',side)

smR:set_transition(up,'down',side)

function entry()
  smR:entry()
--  smL:entry()
end

function update()
  -- Update the joint angles and body RPY
  boxercm.set_body_rpy( libboxer.get_torso_orientation() );
  qL,qR = libboxer.get_arm_angles();
  boxercm.set_body_qLArm( qL );
  boxercm.set_body_qRArm( qR );
  smR:update();
--  smL:update();
end

function exit()
  smR:exit();
--  smL:exit();
end

function event(e)
  smR:add_event(e);
--  smL:add_event(e);
end

function init(forPlayer)
  if( forPlayer ) then
    boxercm = require('boxercm'..forPlayer)
  else
    boxercm = require 'boxercm'
  end
  -- For debugging
  smR:set_state_debug_handle(boxercm.set_fsm_stateR);
--  smL:set_state_debug_handle(boxercm.set_fsm_stateL);
--  boxercm.set_fsm_state('side')

  for i,v in ipairs(smR.states) do
    local s = smR.states[i];
    s['init'](forPlayer);
  end

  libboxer.init(forPlayer);

end
