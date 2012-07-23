module(..., package.seeall);
require('Comm');
require 'Boxer'
require 'boxercm'

require('gcm');
require 'serialization'
require 'Kinematics'
require 'unix'

-- Initialization
--[[
print("My address:",Config.dev.ip_wired)
Comm.init(Config.dev.ip_wired,Config.dev.ip_wired_port)
--]]
print("My address:",Config.dev.ip_wireless)
Comm.init(Config.dev.ip_wireless,Config.dev.ip_wireless_port);

teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
nPlayers = gcm.get_game_nplayers();

msgTimeout = Config.team.msgTimeout or 2;
states = {};
state = {};

if( Config.game.playerID==1 ) then
  require 'primecm'
  print('Using the PrimeSense for control!')  
  ps = true;
end

function entry()
  Boxer.entry()
end

function recv_msgs()
  while (Comm.size() > 0) do 
    t = serialization.deserialize(Comm.receive());
    if (t and (t.id) and (t.id ~= playerID)) then
      t.tReceive = unix.time();
      states[t.id] = t;
    end
  end
end

function update()
  if( ps ) then -- We have a primesense
    Boxer.update()
--    print('Boxer state: ',boxercm.get_fsm_state())
--    print()
    if( primecm.get_skeleton_found() ) then
      send_body();
    end
  else
    recv_msgs();

    -- Check when we last updated
    if( not states[1] or (unix.time() - states[1].tReceive > msgTimeout) ) then
      boxercm.set_body_enabled( 0 );
      return;
    end

    boxercm.set_body_enabled( 1 ); -- 2 is companion mode
    boxercm.set_body_velocity( states[1].vel );
    boxercm.set_body_punchL( states[1].pL );
    boxercm.set_body_punchR( states[1].pR );
    boxercm.set_body_qLArm( states[1].qR );
    boxercm.set_body_qRArm( states[1].qR );
    boxercm.set_body_rpy( states[1].rpy );
  end

end

function send_body()

  -- Organize the data
  state = {};
  state.pst = primecm.get_skeleton_timestamp();
  state.id = playerID;
  state.vel = boxercm.get_body_velocity();
  state.pL = boxercm.get_body_punchL();
  state.pR = boxercm.get_body_punchR();
  state.qL = boxercm.get_body_qLArm();
  state.qR = boxercm.get_body_qRArm();
  state.rpy = boxercm.get_body_rpy();

  -- Burst mode
  local ret = Comm.send( serialization.serialize(state) );
  ret = Comm.send( serialization.serialize(state) );
  ret = Comm.send( serialization.serialize(state) );
  ret = Comm.send( serialization.serialize(state) );
  print('Sent '..ret..' bytes',serialization.serialize(state))

end

function exit()
  Boxer.exit();
end
