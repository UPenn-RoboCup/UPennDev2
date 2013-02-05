module(..., package.seeall);
require('Comm');

require('gcm');
require 'serialization'
require 'unix'
wired = true;
ps = false;

if (string.find(Config.platform.name,'Webots')) then
  print('TeamBox: On webots!')
  webots = true;
else
  print('TeamBox: Real robot!')
end

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
    ps = true;
    -- Require the primecm modules
    for i=1,nPlayers do
      bc[i] = require('boxercm'..i)
      print('Requiring ',i,bc[i])
    end
  else
    require 'boxercm'
  end
end

function recv_msgs()
  while (Comm.size() > 0) do 
    t = serialization.deserialize(Comm.receive());
print('recv '..t.id)
    if (t and (t.id) and (t.id == playerID)) then
      t.tReceive = unix.time();
      states[t.id] = t;
    end
  end
end

function update()
  if( webots ) then
    return;
  end

  if( ps ) then -- We have a primesense
    for i=1,nPlayers do
      if( bc[i].get_body_enabled()>0 ) then
        send_body(i);
      end
    end
  else
    recv_msgs();

    -- Check when we last updated
    if( not states[playerID] or (unix.time() - states[playerID].tReceive > msgTimeout) ) then
      boxercm.set_body_enabled( 0 );
      return;
    end

    boxercm.set_body_enabled( 1 ); -- 2 is companion mode
    --    boxercm.set_body_velocity( states[1].vel );
    boxercm.set_body_punchL( states[playerID].pL );
    boxercm.set_body_punchR( states[playerID].pR );
    boxercm.set_body_qLArm( states[playerID].qL );
    boxercm.set_body_qRArm( states[playerID].qR );
    boxercm.set_body_rpy( states[playerID].rpy );
  end

end

function send_body( forPlayer )

  -- Organize the data
  state = {};
  state.id = forPlayer;
  --  state.vel = boxercm.get_body_velocity();
  bcm = bc[forPlayer];
  state.pL = bcm.get_body_punchL();
  state.pR = bcm.get_body_punchR();
  state.qL = bcm.get_body_qLArm();
  state.qR = bcm.get_body_qRArm();
  state.rpy = bcm.get_body_rpy();

  -- Burst mode
  --[[
  local ser = serialization.serialize(state)
  local ret = Comm.send( ser );
  ret = Comm.send( ser );
  ret = Comm.send( ser );
  ret = Comm.send( ser );
  --]]

  local ser = serialization.serialize(state)
  local ret = Comm.send( ser, #ser );
  ret = Comm.send( ser, #ser );
  ret = Comm.send( ser, #ser );
  ret = Comm.send( ser, #ser );
  
  print('Sent '..forPlayer..': '..ret..' bytes',ser)
end

function exit()
  Boxer.exit();
end
