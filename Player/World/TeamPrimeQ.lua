module(..., package.seeall);
require('Comm');
require 'primecm'; -- Sending and receiving Kinect Data
require('gcm');
require 'serialization'
require 'Kinematics'
require 'unix'

-- Initialization
print("My address:",Config.dev.ip_wireless)
Comm.init(Config.dev.ip_wireless,54321);
teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
msgTimeout = Config.team.msgTimeout;
states = {};
state = {};

if( Config.game.playerID==1 ) then
  print('Team: Using the PrimeSense for control!')  
  require 'primecm'    
  ps = true;
end

function entry()
  print('Relaying pure joint information!');
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
    if( primecm.get_skeleton_found()[1]==1 ) then
      send_body();
    end
  else
    recv_msgs();
    if( states[1] ) then
      -- Push state 0 to the joint space
      primecm.set_joints_qLArm( states[1].la );
      primecm.set_joints_qRArm( states[1].ra );
      primecm.set_joints_rpy( states[1].rpy );
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
  if( t_ps == t_last_ps ) then
    return;
  end
  t_last_ps = t_ps;
  --  print('Time differences',t-t0,t_ps);

  local arms = libboxer.get_arm_angles() or {{0,0,0},{0,0,0}};
  left_arm = arms[1] or left_arm or {0,0,0};
  right_arm = arms[2] or right_arm or {0,0,0};
  local rpy = libboxer.get_torso_orientation() or vector.new({0,0,0});
--  print(string.format('RPY: %.1f %.1f %.1f\n',unpack(180/math.pi*rpy)))
--  walk.upper_body_override(left_arm,right_arm,rpy)

  -- Send the data
  state = {};
  state.t = timestamp;
  state.tid = teamID;
  state.id = playerID;
  state.la = left_arm;
  state.ra = right_arm;
  state.rpy = rpy;
  local ret = Comm.send( serialization.serialize(state) );
  ret = Comm.send( serialization.serialize(state) );
  ret = Comm.send( serialization.serialize(state) );

--[[
t_send = unix.time()
t_diff = t_send - (t_last or 0);
t_last = t_send;
print("Sending at ",1/t_diff)
--]]

--  print('Sent ',ret,'bytes',serialization.serialize(state))

  -- Keep in SHM, too, for local Webots
  -- primecm.set_joints_qLArm( qLArm );
  -- primecm.set_joints_qRArm( qRArm );
end

