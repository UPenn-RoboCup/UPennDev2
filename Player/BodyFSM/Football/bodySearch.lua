module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require 'Kinematics'
require 'vector'
ps = false;

if( Config.stretcher.primesense and Config.game.playerID==1 ) then
  print('Using the PrimeSense for control!')  
  require 'primecm'    
  ps = true;
end

t0 = 0;
timeout = 10.0;

-- turn velocity
vSpin = 0.3;
direction = 1;
-- maximum walk velocity
maxStep = 0.04;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();

  if( not ps ) then  
    -- set turn direction to last known ball position
    stretcher = wcm.get_stretcher();
    if (stretcher.y > 0) then
      direction = 1;
    else
      direction = -1;
    end
  end

  walk.stop();

end

function update()
  local t = Body.get_time();

  if( ps ) then
    if( primecm.get_skeleton_found()==1 ) then
      --print('Updating via PrimeSense')
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
      print( 'Desired arm position: ', unpack(get_scaled_prime_arm(0)) );
      qRArm = Kinematics.inverse_arm(get_scaled_prime_arm(1));
      qLArm = Kinematics.inverse_arm(get_scaled_prime_arm(0));
      if(qRArm) then
        qRArm[2] = -1 * qRArm[2]; 
        Body.set_rarm_command( qRArm );
      end
      if(qLArm) then
        qLArm[2] = -1 * qLArm[2];
        Body.set_larm_command( qLArm );
      end

      --walk.set_velocity( vx, vy, va );
    else
      print('User not found...')
      walk.set_velocity( 0,0,0 );      
    end
  else
    stretcher = wcm.get_stretcher();    
    -- search/spin until the ball is found
    walk.set_velocity(0, 0, direction*vSpin);

    if (t - stretcher.t < 0.1) then
      return "stretcher";
    end
    if (t - t0 > timeout) then
      return "timeout";
    end

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
    e2w = primecm.get_position_ElbowL() - primecm.get_position_WristL();
    s2e = primecm.get_position_ShoulderL() - primecm.get_position_WristL();
    s2w = primecm.get_position_ShoulderL() - primecm.get_position_WristL();
  else
    e2w = primecm.get_position_ElbowR() - primecm.get_position_WristR();
    s2e = primecm.get_position_ShoulderR() - primecm.get_position_WristR();
    s2w = primecm.get_position_ShoulderR() - primecm.get_position_WristR();
  end
  arm_len = vector.norm( e2w ) + vector.norm( s2e );
  offset = s2w * (.18 / arm_len);
  -- Change to correct coordinates for OP
  return {offset[3],offset[1],offset[2]}; -- z is OP x, x is OP y, y is OP z
end
