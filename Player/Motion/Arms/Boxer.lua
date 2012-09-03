module(..., package.seeall);

require 'fsm'
require 'util'
require 'side'
require 'fore'
require 'up'
require 'libboxer'

nPlayers = Config.game.nPlayers;
nPlayers = 2

arm_states = {side,fore,up}

bc = {};
smrs = {}
smls = {}
for i=1,nPlayers do
	bc[i] = require('boxercm'..i)
	boxercm = bc[i];
	boxercm.set_fsm_stateR('side')
	boxercm.set_fsm_stateL('side')
	
	-- Init the arm FSMs
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
	
	smR:set_state_debug_handle(boxercm.set_fsm_stateR);
	smL:set_state_debug_handle(boxercm.set_fsm_stateL);
	
	-- Add to the array
	smrs[i] = smR
	smls[i] = smL
end

function entry(playerID)
	for s = 1,#arm_states do
		arm_states[s].playerID = playerID
	end

	smR = smrs[playerID]
	smL = smls[playerID]
	smR:entry(playerID)
	smL:entry(playerID)
end

function update(playerID)
	boxercm = bc[playerID];
	smR = smrs[playerID]
	smL = smls[playerID]
	for s = 1,#arm_states do
		arm_states[s].playerID = playerID
	end
	
	if( libboxer.check_enabled(playerID) ) then
		boxercm.set_body_enabled( 1 );
		boxercm.set_body_t( unix.time() );
		-- Update the joint angles and body RPY
		boxercm.set_body_rpy( libboxer.get_torso_orientation(playerID) );
		qL,qR = libboxer.get_arm_angles(playerID);
		if( qL ) then
			boxercm.set_body_qLArm( qL );
		end
		if( qR ) then
			boxercm.set_body_qRArm( qR );
		end
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

function exit(playerID)
	smR = smrs[playerID]
	smL = smls[playerID]
	for s = 1,#arm_states do
		arm_states[s].playerID = playerID
	end
	smR:exit();
	smL:exit();
	boxercm.set_body_enabled( 0 );
end

function event(playerID,e)
	for s = 1,#arm_states do
		arm_states[s].playerID = playerID
	end
	smR = smrs[playerID]
	smL = smls[playerID]
	smR:add_event(e);
	smL:add_event(e);
end