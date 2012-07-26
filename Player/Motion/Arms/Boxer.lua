module(..., package.seeall);

require 'fsm'
require 'util'
require 'side'
require 'fore'
require 'up'
require 'libboxer'

arm_states = {side,fore,up}

smR = fsm.new(unpack(arm_states)); -- right hand following.
smR:set_transition(side,'forward',fore)
smR:set_transition(side,'up',up)
smR:set_transition(fore,'up',up)
smR:set_transition(fore,'down',side)
smR:set_transition(up,'down',side)

smL = fsm.new(unpack(arm_states)); -- left hand following.
smL:set_transition(side,'forward',fore)
smL:set_transition(side,'up',up)
smL:set_transition(fore,'up',up)
smL:set_transition(fore,'down',side)
smL:set_transition(up,'down',side)

function entry()
  smR:entry()
  smL:entry()
end

function update()
  -- If available data available
  if( libboxer.check_enabled() ) then
    boxercm.set_body_enabled( 1 );
    boxercm.set_body_t( unix.time() );
    -- Update the joint angles and body RPY
    boxercm.set_body_rpy( libboxer.get_torso_orientation() );
    qL,qR = libboxer.get_arm_angles();
    boxercm.set_body_qLArm( qL );
    boxercm.set_body_qRArm( qR );
  else
    boxercm.set_body_enabled( 0 );
  end
  -- Iterate through the states to prepare them for do the right hand
  for s = 1,#arm_states do
    arm_states[s].myhand = 'right'
  end
  smR:update();

  -- Iterate through the states to prepare them for do the left hand
  for s = 1,#arm_states do
    arm_states[s].myhand = 'left'
  end
  smL:update()

  -- Debug
  --print('R Boxer state: ',boxercm.get_fsm_stateR())
  --print('L Boxer state: ',boxercm.get_fsm_stateL())
end

function exit()
  smR:exit();
  smL:exit();
end

function event(e)
  smR:add_event(e);
  smL:add_event(e);
end

function init(forPlayer)
  if( forPlayer ) then
    boxercm = require('boxercm'..forPlayer)
  else
    boxercm = require 'boxercm'
  end
  -- For debugging
  smR:set_state_debug_handle(boxercm.set_fsm_stateR);
  smL:set_state_debug_handle(boxercm.set_fsm_stateL);
  boxercm.set_fsm_stateR('side')
  boxercm.set_fsm_stateL('side')

  for s = 1,#arm_states do
    arm_states[s].init(forPlayer)
  end

  libboxer.init(forPlayer);

end
