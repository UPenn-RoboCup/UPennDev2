module(..., package.seeall);
require 'primecm'
if( Config.game.playerID==1 ) then
require 'handfilter'
require 'handfilter2'
end
require 'vector'

-- Global variables to track each boxer
last_vx = {0,0};
last_vy = {0,0};
last_va = {0,0};
vx = {0,0};
vy = {0,0};
va = {0,0};
pRight = {false,false};
pLeft = {false,false};
uRight = {false,false};
uLeft = {false,false};
last_t_punch = {0,0};
last_dpunch = {0,0};
handvelL = {0,0};
handvelR = {0,0};

playerID = Config.game.playerID;

function convert_punch(current_stance)
  if(playerID==1) then
    print('not for the master!');
    return 0;
  end
  local raw_punch = primecm.get_joints_punch();
  raw_punch = raw_punch[playerID-1];
  if (raw_punch<1 or raw_punch>4) then
    return 0;
  end

  local real_punch = 0;
  local left_stance = current_stance==1;
  local right_stance = current_stance==2;

  --  print('raw_punch/stance',raw_punch,right_stance,left_stance)
  if( (raw_punch==1 and left_stance) or(raw_punch==3 and right_stance) ) then
    real_punch = 2; -- switch punch
  elseif( (raw_punch==2 and left_stance) or(raw_punch==4 and right_stance) ) then
    real_punch = 3; -- uppercut
  elseif( (raw_punch==1 and right_stance) or(raw_punch==3 and left_stance) ) then
    real_punch = 1; -- jab
  end
  return real_punch;
end

function get_punch( userNum )
  if(pRight[userNum]) then
    return 1;
  elseif(uRight[userNum]) then
    return 2;
  elseif(pLeft[userNum]) then
    return 3;
  elseif(uLeft[userNum]) then
    return 4;
  else
    return 0;
  end
end

function update_walk( userNum )

  vx[userNum] = 0;
  vy[userNum] = 0;
  va[userNum] = 0;

  local velLimit = 0.4;
  if( handvelL[userNum]>velLimit or handvelR[userNum]>velLimit ) then
    va[userNum] = 0;
    vx[userNum] = 0;
    vy[userNum] = 0;
    return;
  end

  -- Ensure we are confident with our estimates
  if( userNum==1 ) then
    sLc = primecm.get_confidence_ShoulderL();
    sRc = primecm.get_confidence_ShoulderR();
    hc = primecm.get_confidence_Head();
    tc = primecm.get_confidence_Torso();
  elseif( userNum==2 ) then
    sLc = primecm.get_confidence2_ShoulderL();
    sRc = primecm.get_confidence2_ShoulderR();
    hc = primecm.get_confidence2_Head();
    tc = primecm.get_confidence2_Torso();
  end
  if( sLc[1]==0 or sRc[1]==0 or hc[1]==0 or tc[1]==0 ) then
    return;
  end
  -- print('Conf: ',sLc,sRc,hc,tc)

  -- Calculate velocities

  local t = primecm.get_skeleton_timestamp();
  -- Head to Torso
  if( userNum==1 ) then
    h2t = primecm.get_position_Head() - primecm.get_position_Torso();
    sL2sR = primecm.get_position_ShoulderL() - primecm.get_position_ShoulderR();
  elseif( userNum==2 ) then
    h2t = primecm.get_position2_Head() - primecm.get_position2_Torso();
    sL2sR = primecm.get_position2_ShoulderL() - primecm.get_position2_ShoulderR();
  end

  local offset_x = 0.002;--0;--0.02; -- Center
  local beta = 0.8;
  last_vx[userNum] = vx[userNum];
  local reading = (-1 * h2t[3] / 2) - offset_x;
  vx[userNum] = beta*reading + (1-beta)*last_vx[userNum];

  local offset_y = 0;
  last_vy[userNum] = vy[userNum];
  reading = (-1 * h2t[1] / 3) - offset_y;
  vy[userNum] = beta*reading + (1-beta)*last_vy[userNum];

  local beta = 0.9;
  last_va[userNum] = va[userNum];
  reading = math.atan2( sL2sR[1],sL2sR[3] ) + math.pi/2;
  va[userNum] = beta*reading + (1-beta)*last_va[userNum];

  -- Clamp and deadband the velocities
  local maxStep = 0.06;
  local deadband = 0.01;
  if(vx[userNum]>maxStep) then
    vx[userNum] = maxStep;
  elseif(vx[userNum]<-1*maxStep) then
    vx[userNum] = -1*maxStep;
  end
  if(vy[userNum]>maxStep) then
    vy[userNum] = maxStep;
  elseif(vy[userNum]<-1*maxStep) then
    vy[userNum] = -1*maxStep;
  end

  -- Deadband can prevent drift
  if(vx[userNum]<deadband and vx[userNum]>-1*deadband) then
    vx[userNum] = 0;
  end
  if(vy[userNum]<deadband and vy[userNum]>-1*deadband) then
    vy[userNum] = 0;
  end

  --[[
  deadband = 0.05;
  if(va[userNum]<deadband and va[userNum]>-1*deadband) then
  va[userNum] = 0;
  end
  --]]

end

function update_punch( userNum )

  pRight[userNum] = false;
  pLeft[userNum] = false;
  uRight[userNum] = false;
  uLeft[userNum] = false;

  -- TODO: Need to check the confidence values!
  if( userNum==1 ) then
    e2hL = primecm.get_position_ElbowL() - primecm.get_position_HandL();
    s2eL = primecm.get_position_ShoulderL() - primecm.get_position_ElbowL();
    s2hL = primecm.get_position_ShoulderL() - primecm.get_position_HandL();
    e2hR = primecm.get_position_ElbowR() - primecm.get_position_HandR();
    s2eR = primecm.get_position_ShoulderR() - primecm.get_position_ElbowR();
    s2hR = primecm.get_position_ShoulderR() - primecm.get_position_HandR();
  elseif( userNum==2 ) then
    e2hL = primecm.get_position2_ElbowL()    - primecm.get_position2_HandL();
    s2eL = primecm.get_position2_ShoulderL() - primecm.get_position2_ElbowL();
    s2hL = primecm.get_position2_ShoulderL() - primecm.get_position2_HandL();
    e2hR = primecm.get_position2_ElbowR()    - primecm.get_position2_HandR();
    s2eR = primecm.get_position2_ShoulderR() - primecm.get_position2_ElbowR();
    s2hR = primecm.get_position2_ShoulderR() - primecm.get_position2_HandR();
  end
  -- Change to correct coordinates for OP
  local arm_lenL = vector.norm( e2hL ) + vector.norm( s2eL );
  local arm_lenR = vector.norm( e2hR ) + vector.norm( s2eR );
  local left_hand  = vector.new({s2hL[3],s2hL[1],s2hL[2]}) / arm_lenL; -- z is OP x, x is OP y, y is OP z
  local right_hand = vector.new({s2hR[3],s2hR[1],s2hR[2]}) / arm_lenR;

  -- Check if hands are high for a while
  --[[
  if( right_hand[3]<-.5 and left_hand[3]<-.5 ) then
  pause[userNum] = pause[userNum]+1;
  else
  pause[userNum] = 0;
  end
  --]]
  -- Scale to OP
  local left_hand_op = left_hand * .189;
  local right_hand_op = right_hand * .189;

  -- Update the hand filter
  -- filtered_ball [x y vx vy ep evp]
  -- TODO: May need to check confidence, actually..
  local t = primecm.get_skeleton_timestamp();
  local dFrame = 0;
  local fps = 30;
  if( last_t_punch[userNum] ) then 
    dt = t - last_t_punch[userNum];
    dFrame = math.floor( fps*dt+.5 );
    --print('df: ',dFrame)
  end
  last_t_punch[userNum] = t;
  -- Need to store these for each robot
  if( userNum==1 ) then
    xR, zR, vxR, vzR, epR, evpR  = 
    handfilter.get_right_hand(right_hand[1],right_hand[3],dFrame);
    xL, zL, vxL, vzL, epL, evpL  = 
    handfilter.get_left_hand(left_hand[1],left_hand[3],dFrame);
    --print('hand: ',xR, zR, vxR, vzR, epR, evpR)
  elseif(userNum==2) then
    xR, zR, vxR, vzR, epR, evpR  = 
    handfilter2.get_right_hand(right_hand[1],right_hand[3],dFrame);
    xL, zL, vxL, vzL, epL, evpL  = 
    handfilter2.get_left_hand(left_hand[1],left_hand[3],dFrame);
    --print('hand: ',xR, zR, vxR, vzR, epR, evpR)
  end


  if( dFrame==1 ) then
    --print('checking ',userNum)
    vxR = vxR * fps;vzR = vzR*fps;
    vxL = vxL * fps;vzL = vzL*fps;

    -- Left Hand
    if( zL<-0.3 and vzL<-1 ) then
      uLeft[userNum] = true;
      print('^^'..userNum..'^^Uppercut Left! @ ',t)
    else 
      if( xL>.4 and vxL>.5 and math.abs(zL)<.1 and math.abs(vzL)<2 ) then
        pLeft[userNum] = true;
        print('**'..userNum..'**Punch Left! @ ',t)
      end
    end

    -- Right Hand
    if( zR<-0.3 and vzR<-1 ) then
      uRight[userNum] = true;
      print('^^'..userNum..'^^Uppercut Right! @ ',t)
    else 
      if( xR>.4 and vxR>.5 and math.abs(zR)<.1 and math.abs(vzR)<2 ) then
        pRight[userNum] = true;
        print('**'..userNum..'**Punch Right! @ ',t,vzR)
      end
    end
  else
    --print('punch data: ',pLeft,uLeft,pRight,uRight)
  end -- dFrame is 1

  -- Upate our last punch time...
  if( pLeft[userNum] or uLeft[userNum] or pRight[userNum] or uRight[userNum] ) then
    -- Don't fire if we just punched... (Wait a quarter second)
    local dPunchTime = t - last_dpunch[userNum];
    --print(userNum..': '..t..'/'..last_dpunch[userNum]..' dPunchTime('..userNum..'): ',dPunchTime);
    last_dpunch[userNum] = t;
    if( dPunchTime<.25 ) then
      print('negating punch!')
      pRight[userNum] = false;
      pLeft[userNum] = false;
      uRight[userNum] = false;
      uLeft[userNum] = false;
    else
      -- print(t..' punch! ',pLeft,uLeft,pRight,uRight)
    end
  end

  handvelL[userNum] = math.sqrt( vxL^2+vzL^2 );
  handvelR[userNum] = math.sqrt( vxR^2+vzR^2 );
end

-- Updates the filters and the shm
function update( userNum )
  local using_ps = primecm.get_skeleton_enabled();
  if( using_ps==1 ) then
    local found = primecm.get_skeleton_found();
    --if( found[userNum]>0 ) then
    if( found[userNum]>0 and found[userNum]<3 ) then --Limit no.(users) to two
      --found = true;
      update_punch( userNum );
      update_walk( userNum );
      --      local mypunch = {0,0};
      local mypunch = primecm.get_joints_punch( );
      mypunch[userNum] = get_punch(userNum);
      primecm.set_joints_punch( mypunch );
      --       print('Velocity ',vx[userNum],vy[userNum],va[userNum])
      if( userNum==1 ) then      
        primecm.set_skeleton_velocity({vx[userNum],vy[userNum],va[userNum]});
      elseif( userNum==2) then
        primecm.set_skeleton_velocity2({vx[userNum],vy[userNum],va[userNum]});
      end
    end
  end

end

-- 
-- General Utilities
--
function get_arm_angles()
  saL,elL = get_scaled_prime_arm(0);
  saR,elR = get_scaled_prime_arm(1);
  --[[
  if( not saL or not saR) then
    return;
  end
  --]]
  qLArm = nil;
  qRArm = nil;
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

  
  return {qLArm,qRArm};
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

  -- Grab Confidence
  sLc = primecm.get_confidence_ShoulderL();
  sRc = primecm.get_confidence_ShoulderR();
  nc = primecm.get_confidence_Neck();
  nc = primecm.get_confidence_Head();
  tc = primecm.get_confidence_Torso();

  if( sLc[1]==0 or sRc[1]==0 or nc[1]==0 or tc[1]==0 ) then
    --print('Confidence issues...')
    --print(sLc[1],sRc[1],nc[1],tc[1])
    return;
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
