module(..., package.seeall);
require('Comm');
require 'primecm'; -- Sending and receiving Kinect Data
require('gcm');
require 'serialization'
require 'Kinematics'
require 'unix'

-- Initialization
print("My address:",Config.dev.ip_wired)
Comm.init(Config.dev.ip_wired,54321);
teamID   = gcm.get_team_number();
playerID = gcm.get_team_player_id();
msgTimeout = Config.team.msgTimeout;
states = {};
state = {};

if( Config.stretcher.primesense and Config.game.playerID==1 ) then
  print('Using the PrimeSense for control!')  
  require 'primecm'    
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
    if( primecm.get_skeleton_found()==1 ) then
      send_body();
    end
  else
    recv_msgs();
    if( states[0] ) then
      -- Push state 0 to the joint space
      primecm.set_joints_qLArm( states[0].qLArm );
      primecm.set_joints_qRArm( states[0].qRArm );
    end
  end

end

function send_body()
  if( t_last_ps==0 ) then
    t0 = unix.time();
  end
  t_ps = primecm.get_skeleton_timestamp();
  found_ps = primecm.get_skeleton_found();
  if( t_ps == t_last_ps or found_ps == 0) then
    return;
  end
  t_last_ps = t_ps;
--  print('Time differences',t-t0,t_ps);

  qRArm = Kinematics.inverse_arm(get_scaled_prime_arm(1));
  qLArm = Kinematics.inverse_arm(get_scaled_prime_arm(0));

  if(qRArm) then
    qRArm[2] = -1 * qRArm[2]; 
    --Body.set_rarm_command( qRArm );
  end
  if(qLArm) then
    qLArm[2] = -1 * qLArm[2];
    --Body.set_larm_command( qLArm );
  end

  -- Send the data
  state = {};
  state.t = timestamp;
  state.tid = teamID;
  state.id = 0;--playerID;
  state.found = found_ps;
  state.qRArm = qRArm;
  state.qLArm = qLArm;
  local ret = Comm.send( serialization.serialize(state) );
  print('Sent ',ret,'bytes',serialization.serialize(state))
end

function get_scaled_prime_arm( arm ) --left is 0
  --%const double upperArmLength = .060;  //OP, spec
  --%const double lowerArmLength = .129;  //OP, spec
  --op_arm_len = .189;
  if(arm==0) then
    e2h = primecm.get_position_ElbowL() - primecm.get_position_HandL();
    s2e = primecm.get_position_ShoulderL() - primecm.get_position_ElbowL();
    s2h = primecm.get_position_ShoulderL() - primecm.get_position_HandL();
  else
    e2h = primecm.get_position_ElbowR() - primecm.get_position_HandR();
    s2e = primecm.get_position_ShoulderR() - primecm.get_position_ElbowR();
    s2h = primecm.get_position_ShoulderR() - primecm.get_position_HandR();
  end
  arm_len = vector.norm( e2h ) + vector.norm( s2e );
  offset = s2h * (.189 / arm_len);
  -- Change to correct coordinates for OP
  return vector.new({offset[3],offset[1],offset[2]}); -- z is OP x, x is OP y, y is OP z
end
