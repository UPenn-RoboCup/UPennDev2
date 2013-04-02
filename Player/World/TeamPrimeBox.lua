module(..., package.seeall);
require('Comm');
--require('Comm2');
require 'primecm'; -- Sending and receiving Kinect Data
require('gcm');
require 'serialization'
require 'Kinematics'
require 'unix'
require 'primecm'
require 'libboxer'

-- Initialization
--[[
print("My address:",Config.dev.ip_wired)
Comm.init(Config.dev.ip_wired,54321);
--]]
--
print("My address:",Config.dev.ip_wireless)
Comm.init(Config.dev.ip_wireless,54321);
--

teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
msgTimeout = Config.team.msgTimeout;
states = {};
state = {};

if( Config.game.playerID==1 ) then
  print('Using the PrimeSense for control!')  
  ps = true;
end

function entry()

end

function recv_msgs()
  while (Comm.size() > 0) do 
    t = serialization.deserialize(Comm.receive());
    if (t and (t.tid) and (t.tid == teamID ) and (t.id) and (t.id ~= playerID)) then
      t.tReceive = unix.time();
      states[t.id] = t;
    end
  end
end

function update()
  if( ps ) then -- We have a primesense
    local found = primecm.get_skeleton_found();
    if(found[1]>0 or found[2]>0 ) then
      send_body();
    end
  else
    recv_msgs();

    if( states[1] ) then
      local punch = {0,0};
      punch[playerID-1] = states[1].punch[playerID-1];
      primecm.set_joints_punch( punch );
--if( playerID==2 ) then
      primecm.set_skeleton_velocity( states[1].vel );
--else
      primecm.set_skeleton_velocity2( states[1].vel2 );
--end
      -- Check when we last updated
      if( unix.time() - states[1].tReceive < 2) then
        primecm.set_skeleton_enabled( 2 ); -- 2 is companion mode
      else
        primecm.set_skeleton_enabled( 0 );
      end
    else
      primecm.set_skeleton_enabled( 0 );
    end
  end

end

function send_body()
  if( t_last_ps==0 ) then
    t0 = unix.time();
  end
  t_ps = primecm.get_skeleton_timestamp();
  found_ps = primecm.get_skeleton_found();
  -- For 2 players
  found_ps = found_ps[1]>0 or found_ps[2]>0;
  if( t_ps == t_last_ps or not found_ps) then
    return;
  end
  t_last_ps = t_ps;

  -- Update via the boxing library
  local vel = primecm.get_skeleton_velocity();
  local vel2 = primecm.get_skeleton_velocity2();
  local punch = primecm.get_joints_punch();
  --print('Punch state:',punch)

  -- Send the data
  state = {};
  state.t = timestamp;
  state.tid = teamID;
  state.id = playerID;
  state.vel = vel;
  state.vel2 = vel2;
  state.punch = punch;

  local ret = Comm.send( serialization.serialize(state) );
--  local ret = Comm2.send( serialization.serialize(state) );

  if( punch[1]>0 or punch[2]>0 ) then
    -- Add a burst of packets to mitigate loss
    local b=0;
    while(b<10) do
      Comm.send( serialization.serialize(state) );
--      Comm2.send( serialization.serialize(state) );
      b = b+1;
    end
    print('Sent ',ret,'bytes',serialization.serialize(state))
  end
end

