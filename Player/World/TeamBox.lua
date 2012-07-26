module(..., package.seeall);
require('Comm');

require('gcm');
require 'serialization'
require 'unix'
wired = false;
ps = false;

-- Initialization
if( wired ) then
  print("My address:",Config.dev.ip_wired)
  Comm.init(Config.dev.ip_wired,Config.dev.ip_wired_port)
else
  print("My address:",Config.dev.ip_wireless)
  Comm.init(Config.dev.ip_wireless,Config.dev.ip_wireless_port);
end

teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
nPlayers = gcm.get_game_nplayers();
-- For testing
nPlayers = 2;
bc = {};

msgTimeout = Config.team.msgTimeout or 2;
states = {};
state = {};

function entry( prime )
  if(prime) then
    print('Using the PrimeSense for control!')  
    -- Require the primecm modules
    for i=1,nPlayers do
      bc[i] = require('boxercm'..i)
      print('Requiring ',i,bc[i])
    end
    ps = true;
  else
    require 'boxercm'
  end
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
    for i=1,nPlayers do
      if( bc[i].get_body_enabled() ) then
        send_body(i);
      end
    end
  else
    recv_msgs();

    -- Check when we last updated
    if( not states[1] or (unix.time() - states[1].tReceive > msgTimeout) ) then
      boxercm.set_body_enabled( 0 );
      return;
    end

    boxercm.set_body_enabled( 1 ); -- 2 is companion mode
    --    boxercm.set_body_velocity( states[1].vel );
    boxercm.set_body_punchL( states[1].pL );
    boxercm.set_body_punchR( states[1].pR );
    boxercm.set_body_qLArm( states[1].qL );
    boxercm.set_body_qRArm( states[1].qR );
    boxercm.set_body_rpy( states[1].rpy );
  end

end

function send_body( forPlayer )

  -- Organize the data
  state = {};
  state.id = 1;
  --  state.vel = boxercm.get_body_velocity();
  bcm = bc[forPlayer];
  state.pL = bcm.get_body_punchL();
  state.pR = bcm.get_body_punchR();
  state.qL = bcm.get_body_qLArm();
  state.qR = bcm.get_body_qRArm();
  state.rpy = bcm.get_body_rpy();

  -- Burst mode
  local ser = serialization.serialize(state)
  local ret = Comm.send( ser );
  ret = Comm.send( ser );
  ret = Comm.send( ser );
  ret = Comm.send( ser );
 print('Sent '..forPlayer..':'..ret..' bytes',ser)

end

function exit()
  Boxer.exit();
end
