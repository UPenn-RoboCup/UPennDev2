module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require 'Kinematics'
require 'vector'
require 'primecm'
ps = false;
beta = 0.8;
qlarm_mirror = vector.zeros(3);
qrarm_mirror = vector.zeros(3);


if( Config.stretcher.primesense and Config.game.playerID==1 ) then
  print('Using the PrimeSense for control!')  
--  require 'primecm'    
  ps = true;
end

t0 = 0;
timeout = 10.0;

t_last_ps = 0;

-- turn velocity
vSpin = 0.3;
direction = 1;
-- maximum walk velocity
maxStep = 0.04;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();
  walk.stop();

end

function update()
  local t = Body.get_time();

  if( ps ) then
    if( primecm.get_skeleton_found()==1 ) then
      update_body(); 
      --walk.set_velocity( vx, vy, va );
    else
      walk.set_velocity( 0,0,0 );      
    end
  else -- We do not have a primesense
    local qlarm_state = primecm.get_joints_qLArm()
    local qrarm_state = primecm.get_joints_qRArm()
    qlarm_mirror = (1-beta)*qlarm_mirror + beta*qlarm_state;
    qrarm_mirror = (1-beta)*qrarm_mirror + beta*qrarm_state;

    --print('Team arm state:',qlarm_state);
    Body.set_larm_command( qlarm_mirror );
    Body.set_rarm_command( qrarm_mirror );
  end

  if (false and t - t0 > timeout) then
      return "timeout";
    end
end

function exit()
end

function get_scaled_prime_arm( arm ) --left is 0
  -- Arm calculations
  --[[
  e2w = positions(indexElbow,:) - positions(indexWrist,:);
  s2e = positions(indexShoulder,:) - positions(indexElbow,:);
  s2w = positions(indexShoulder,:) - positions(indexWrist,:);
  arm_len = sqrt(norm(e2w)) + sqrt(norm(s2e));
  offset = s2w * (op_arm_len / arm_len);
  --]]
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

function kine_map( dArm )

  -- Real numbers
  upperArmLength = .060;
  lowerArmLength = .129;

  qArm = -999 * vector.ones(3);
  -- Law of cosines to find end effector distance from shoulder
  c_sq = dArm[1]^2+dArm[2]^2+dArm[3]^2;
  c = math.sqrt( c_sq );
  print('dArm',dArm,'c',c)  
  if( c>lowerArmLength+upperArmLength ) then
    disp('Distance not reachable!');
    return;
  end
  local tmp = ((upperArmLength^2)+(lowerArmLength^2)-c_sq) / (2*upperArmLength*lowerArmLength);
  if( tmp>1 ) then -- Impossible configuration
    disp('Impossible confirguration!');
    return;
  end
  qArm[3] = math.acos( tmp );
  -- Angle of desired point with the y-axis
  qArm[2] = math.acos( dArm[2] / c );
  -- How much rotation about the y-axis (in the xz plane
  qArm[1] = math.atan2( dArm[3], dArm[1] ) - qArm[3];

  -- Condition for OP default joint position
  qArm[3] = qArm[3] - math.pi;
  qArm[2] = qArm[2] - math.pi/2;
  qArm[1] = qArm[1] + math.pi;

  qArm = qArm * 180/math.pi;
  return qArm;

end

function update_body()
  if( t_last_ps==0 ) then
    t0 = Body.get_time();
  end
  t_ps = primecm.get_skeleton_timestamp();
  if( t_ps == t_last_ps ) then
    return;
  end
  t_last_ps = t_ps;
--[[
  local t = Body.get_time();
  print('Time differences',t-t0,t_ps);
--]]
  local torso = primecm.get_position_Torso();
  local vx = -1*torso[3] - 240;
  local vy = -1*torso[1] - 200;
  local va = 0;
  scale = math.min(maxStep/math.sqrt(vx^2+vy^2), 1);
  vx = vx * scale;
  vy = vy * scale;
  --print('torso',unpack(torso))
  --print('vel:',vx,vy)
  --print()
  -- Body.set_rarm_command(vector.zeros(3));
  --print( 'Desired arm position: ', unpack(get_scaled_prime_arm(0)) );
  qRArm = Kinematics.inverse_arm(get_scaled_prime_arm(1));
  qLArm = Kinematics.inverse_arm(get_scaled_prime_arm(0));
  --qLArm2 = kine_map( get_scaled_prime_arm(0) );

  if(qRArm) then
    qRArm[2] = -1 * qRArm[2]; 
    Body.set_rarm_command( qRArm );
  end
  if(qLArm) then
    --[[
    print('C++: ',180/math.pi*vector.new(qLArm) )
    print('Lua: ',qLArm2)
    print();
    --]]
    qLArm[2] = -1 * qLArm[2];
    Body.set_larm_command( qLArm );
  end

end
