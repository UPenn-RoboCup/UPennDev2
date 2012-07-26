module(..., package.seeall);

require 'vector'
require 'Kinematics'

playerID = Config.game.playerID;

function init( forPlayer )
  primecm = require('primecm'..forPlayer)  
end

function check_enabled()
  enabled = primecm.get_skeleton_found();
  return enabled;
end

function get_arm_angles()
  saL,elL = get_scaled_prime_arm(0);
  saR,elR = get_scaled_prime_arm(1);
  --[[
  if( not saL or not saR) then
  return;
  end
  --]]
  qLArm = vector.zeros(3);
  qRArm = vector.zeros(3);
  if( saL and math.abs(elL)<=math.pi ) then
    --    print('saL: ', unpack(saL) );
    --print('ElbowL: ',elL);
    qLArm = Kinematics.inverse_arm( saL,vector.ones(1)*elL );
    if(qLArm) then
      qLArm[2] = -1*qLArm[2];
      qLArm[1] = qLArm[1]+elL;
      qLArm[3] = -elL;
      --    print('qL: ', unpack(qLArm) );
    end
  end

  if( saR and math.abs(elR)<=math.pi ) then
    --print('ElbowR: ',elR);    
    qRArm = Kinematics.inverse_arm( saR,vector.ones(1)*elR );
    if(qRArm) then

      qRArm[1] = qRArm[1]+elR;
      qRArm[3] = -elR;
    end
  end

  --  qLArm[1] = qLArm[1]+elL/2+math.pi;
  --  qRArm[1] = qRArm[1]+elR/2+math.pi;
  qLArm = qLArm or vector.zeros(3);
  qRArm = qRArm or vector.zeros(3);

  return qLArm, qRArm;
end

function get_scaled_prime_arm( arm ) --left is 0
  --%const double upperArmLength = .060;  //OP, spec
  --%const double lowerArmLength = .129;  //OP, spec
  --op_arm_len = .189;
  if(arm==0) then
    e2h = primecm.get_position_ElbowL() - primecm.get_position_HandL();
    s2e = primecm.get_position_ShoulderL() - primecm.get_position_ElbowL();
    --s2h = primecm.get_position_ShoulderL() - primecm.get_position_HandL();

    -- Check confidence
    e = primecm.get_confidence_ElbowL();
    s = primecm.get_confidence_ShoulderL();
    h = primecm.get_confidence_HandL();

  else
    e2h = primecm.get_position_ElbowR() - primecm.get_position_HandR();
    s2e = primecm.get_position_ShoulderR() - primecm.get_position_ElbowR();
    --s2h = primecm.get_position_ShoulderR() - primecm.get_position_HandR();

    -- Check confidence
    e = primecm.get_confidence_ElbowR();
    s = primecm.get_confidence_ShoulderR();
    h = primecm.get_confidence_HandR();

  end

  if(e[1]<.5 or s[1]<.5 or h[1]<.5) then
    --print('Not confident!',s,e,h);
    return nil;
  end
  -- ELBOW ANGLE
  el = math.acos( s2e*e2h / (vector.norm(s2e)*vector.norm(e2h) ) )


  e2h = e2h/vector.norm( e2h ) * .129;
  s2e = s2e / vector.norm( s2e ) * .060;
  --arm_len = vector.norm( e2h ) + vector.norm( s2e );
  local offset_raw = s2e+e2h;
  --print('OR: ',offset_raw);
  --[[
  local offset_unit = s2h / arm_len;
  local offset_raw = offset_unit * .189;
  --]]
  -- Filter the offset
  local beta = .9;
  if( not offset ) then
    offset = offset_raw;
  else
    offset = beta*offset_raw + (1-beta)*offset;
  end
  -- Change to correct coordinates for OP
  local op_coord = vector.new({offset[3],-1*offset[1],offset[2]}); -- z is OP x, x is OP y, y is OP z
  -- If left hand rev the y
  if( arm==0 ) then
    op_coord[2] = -1*op_coord[2];
  end

  if( op_coord[2] < 0 ) then
    op_coord[2] = 0;
  end
  return op_coord,el;
end

function get_torso_orientation()

  rpy = vector.zeros(3);
  -- Grab Confidence
  sLc = primecm.get_confidence_ShoulderL();
  sRc = primecm.get_confidence_ShoulderR();
  nc = primecm.get_confidence_Neck();
  nc = primecm.get_confidence_Head();
  tc = primecm.get_confidence_Torso();

  if( sLc[1]==0 or sRc[1]==0 or nc[1]==0 or tc[1]==0 ) then
    --print('Confidence issues...')
    --print(sLc[1],sRc[1],nc[1],tc[1])
    return rpy;
  end

  -- Neck to Waist and Shoulder to Shoulder
  --t2n = primecm.get_position_Neck() - primecm.get_position_Torso();
  t2n = primecm.get_position_Head() - primecm.get_position_Torso();
  s2s = primecm.get_position_ShoulderR() - primecm.get_position_ShoulderL();

  -- These should form an orthonormal basis
  -- Each norm is 1
  t2n = t2n / vector.norm(t2n);
  s2s = s2s / vector.norm(s2s);
  --Check the properties:
  --[[
  if( t2n*s2s > 0.05 ) then
  print('Dot product should be zero: ', n2t*sL2sR);
  end
  --]]
  -- Find the cross product
  chest = cross(s2s,t2n);

  -- Remap the coordinates
  u = vector.new({chest[3],chest[1],chest[2]})
  v = vector.new({s2s[3],s2s[1],s2s[2]})
  w = vector.new({t2n[3],t2n[1],t2n[2]})

  -- Find the closest Orthonormal Matrix
  local matrix = require 'matrix'
  M = matrix.transpose( matrix{u,v,w} )
  -- Real Rotation Matrix
  R = M*(matrix.transpose(M)*M)^-1/2

  -- Grab Euler Angles for SJ
  --http://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation#Roll-Pitch-Yaw_Angles
  r = math.atan2(R[3][2],R[3][3])
  y = math.atan2(R[2][1],R[1][1])
  p = math.atan2(-1*R[3][1],math.cos(y)*R[1][1]+math.sin(y)*R[2][1])

  -- Clamp them
  r = util.procFunc(r,0,10*math.pi/180)
  p = util.procFunc(p,0,20*math.pi/180)
  y = util.procFunc(y,0,30*math.pi/180)

  rpy = vector.new({r,p,y})
  --print("RPY: ",unpack(180/math.pi* rpy) )
  return rpy;
end

function cross(v1,v2)
  v3 = vector.zeros(3);
  v3[1] =   ( (v1[2] * v2[3]) - (v1[3] * v2[2]) )
  v3[2] = - ( (v1[1] * v2[3]) - (v1[3] * v2[1]) )
  v3[3] =   ( (v1[1] * v2[2]) - (v1[2] * v2[1]) )
  return v3; 
end
