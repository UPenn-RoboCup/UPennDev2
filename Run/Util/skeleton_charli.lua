module(... or "",package.seeall)
require 'unix'
matrix = require 'matrix'
require('vector');

function entry( mylogs )

  if(mylogs and #mylogs>0) then
    print("Running Skeleton from logs for "..#mylogs.." users.")
    logs = mylogs
    nPlayers = #logs
    timestamp0 = 0
    count = 0
    timestamp = timestamp0
    t_update = unix.time()
  else
    require 'primesense'
    nPlayers = 1
    timestamp0 = unix.time();
  end

  -- Require the primecm modules
  pc = {};
  for i=1,nPlayers do
    pc[i] = require('primecm'..i)
  end
  jointNames = pc[playerID].jointNames

  -- Filtering variables
  last_pos = {};
  last_t = {}

  -- Logging Variables
  logging = false
  x = vector.zeros(#jointNames);
  y = vector.zeros(#jointNames);
  z = vector.zeros(#jointNames);
  posconf = vector.zeros(#jointNames);
  rotconf = vector.zeros(#jointNames);
  saveCount = 0;
  filename = string.format("/tmp/prime_%03d.raw", saveCount);

end

function toggle_logging()
  if(not logging) then
    logging = true
    if not f then
      f = io.open(filename, "w+");
      f:write('log={\n');
      print('Starting logging')
    else
      print('Resuming logging')
    end
  else
    logging = false
    print('Pausing logging')
  end
  return logging
end

function update()
  count = count+1
  if(logs) then
    return update_logs()
  else
    return update_primesense()
  end
end

function update_logs()
  if(count>#logs[1]) then
    return false
  end
  timestamp_last = timestamp
  timestamp = logs[1][count].t - logs[1][1].t;
  timestamp_diff = timestamp - timestamp_last
  for pl=1,nPlayers do
    log = logs[pl];
    for i,v in ipairs(jointNames) do
      pos = { log[count].x[i],log[count].y[i],log[count].z[i] };
      confidence = { log[count].posconf[i],log[count].rotconf[i] };
      primecm = pc[pl];
      primecm['set_position_'..v]( pos );
      primecm['set_confidence_'..v]( confidence );
    end
    primecm.set_skeleton_found( 1 );
    primecm.set_skeleton_timestamp( timestamp-timestamp0 );
  end

  -- Timing	
  t_update_diff = unix.time() - t_update;
  unix.usleep( 1e6*math.max(0,timestamp_diff-t_update_diff) );
  t_update = unix.time();

  return true
end

function update_primesense()
  local ret = primesense.update_joints();
  timestamp = unix.time();

  -- Grab torso positions
  local center = {};
  for pl=1,nPlayers do
    local pos, rot, confidence, active = primesense.get_jointtables(pl,3);
    center[pl] = pos[1];
  end

  -- Grab raw positions
  for pl=1,nPlayers do
    -- Choose the right player's SHM
    local primecm = pc[pl];
    for i,v in ipairs(pc[playerID].jointNames) do
      pos, rot, confidence, active = primesense.get_jointtables(pl,i);
      -- Convert Positions to meters
      pos = vector.new(pos)/1000;

      beta = .5;
      beta = .25;
      if last_pos[i] then
        -- Perform filters.
        -- First is the outlier
        if vector.norm(pos-last_pos[i])>.1 and last_t[i]<.5 then
          pos = last_pos[i]
        else
          pos = beta*pos + (1-beta)*last_pos[i];
          pos = round2(pos,3);
          last_t[i] = timestamp
        end
      end

      last_pos[i] = pos;
      if(logging and pl==1) then
        x[i] = pos[1];
        y[i] = pos[2];
        z[i] = pos[3];
        posconf[i] = confidence[1];
        rotconf[i] = confidence[2];
      end

      -- Set in memory
      primecm['set_position_'..v]( pos );
      primecm['set_orientation_'..v]( rot );
      primecm['set_confidence_'..v]( confidence );
      primecm.set_skeleton_found( active );
      primecm.set_skeleton_timestamp( timestamp-timestamp0 );
    end
  end

  -- Handle log file writing
  if( logging ) then
    -- Write the log file
    f:write( string.format("{t=%f,",timestamp-timestamp0)  );
    f:write( string.format("x={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(x))  );
    f:write( string.format("y={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(y))  );
    f:write( string.format("z={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(z))  );
    f:write( string.format("posconf={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f},",unpack(posconf))  );
    f:write( string.format("rotconf={%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f}",unpack(rotconf))  );
    f:write('},\n');
    if(count%500 == 0)then
      print('Saving Log '..saveCount..'...')
      saveCount = saveCount + 1;
      -- Close old file, open new
      f:write('}\n');      
      f:close();
      print('Done!')
      filename = string.format("/tmp/prime_%03d.raw", saveCount);
      f = io.open(filename, "w+");
      f:write('log={\n');
    end
  end


  return true
end

function exit()
  for pl=1,#pc do
    print('Disabling user '..pl)
    primecm = pc[pl];
    primecm.set_skeleton_found( 0 );  
  end
end

-- Processing functions
function update_arms()

  -- Left Hand
  e2hL = primecm.get_position_ElbowL() - primecm.get_position_HandL();
  s2eL = primecm.get_position_ShoulderL() - primecm.get_position_ElbowL();
  -- Change to correct coordinates for OP
  -- z is OP x, x is OP y, y is OP z
  e2hL = vector.new({e2hL[3],-1*e2hL[1],e2hL[2]}); 
  s2eL = vector.new({s2eL[3],-1*s2eL[1],s2eL[2]});
  -- Check confidence
  ecL = primecm.get_confidence_ElbowL();
  scL = primecm.get_confidence_ShoulderL();
  hcL = primecm.get_confidence_HandL();

  -- Right Hand
  e2hR = primecm.get_position_ElbowR() - primecm.get_position_HandR();
  s2eR = primecm.get_position_ShoulderR() - primecm.get_position_ElbowR();
  -- Change to correct coordinates for OP
  -- z is OP x, x is OP y, y is OP z
  e2hR = vector.new({e2hR[3],-1*e2hR[1],e2hR[2]}); 
  s2eR = vector.new({s2eR[3],-1*s2eR[1],s2eR[2]}); 
  -- Check confidence
  ecR = primecm.get_confidence_ElbowR();
  scR = primecm.get_confidence_ShoulderR();
  hcR = primecm.get_confidence_HandR();

  -- Elbow calculations
  elL = math.acos( s2eL*e2hL / (vector.norm(s2eL)*vector.norm(e2hL) ) );
  elR = math.acos( s2eR*e2hR / (vector.norm(s2eR)*vector.norm(e2hR) ) );
  -- Take care of the NaN's
  if elL~=elL then
    elL = 0;
  end
  if elR~=elR then
    elR = 0;
  end

  -- Shoulder calculations
  -- XZ plane defines the shoulder pitch
  spL = math.atan2(s2eL[3],s2eL[1]) or 0;
  spR = math.atan2(s2eR[3],s2eR[1]) or 0;
  -- YZ plane defines the shoulder roll
  srL = math.atan2( s2eL[2], math.sqrt(s2eL[1]^2+s2eL[3]^2) ) or 0;
  srR = math.atan2( s2eR[2], math.sqrt(s2eR[1]^2+s2eR[3]^2) ) or 0;

  -- TODO: Arm Yaw
	-- Left
	local zeroTrans1 = Transform.rotY(spL)
	s2eL[4] = 1;
	local pLArm1 = zeroTrans1 * s2eL;
	local zeroTrans2 = Transform.rotZ(-1*srL)
	local pLArm2 = zeroTrans2 * pLArm1;
	e2hL[4] = 1;
	local yawdL = zeroTrans2 * zeroTrans1 * e2hL;
  yawL = math.atan2(yawdL[2],-1*yawdL[3]) or 0;

	-- Right
	local zeroTrans1 = Transform.rotY(spR)
	s2eR[4] = 1;
	local pRArm1 = zeroTrans1 * s2eR;
	local zeroTrans2 = Transform.rotZ(-1*srR)
	local pRArm2 = zeroTrans2 * pRArm1;
	e2hR[4] = 1;
	local yawdR = zeroTrans2 * zeroTrans1 * e2hR;
  yawR = -1*math.atan2(yawdR[2],-1*yawdR[3]) or 0;
--yawR = yawR+math.pi

  -- Organize the vector
  qLArm = vector.new({spL,-1*srL,yawL,-1*elL});
  qRArm = vector.new({spR,-1*srR,yawR,-1*elR});
	
  --print('fk1: ',pLArm1, 'fk2: ',pLArm2 )
	--print('L:', s2eL, 'y',yawdL)
	--print('yawL:',yawL*180/math.pi)
	--print(qLArm*180/math.pi)
	--print()

end

function update_torso()

  -- Default value
  rpy = vector.zeros(3);

  -- Confidence check
  sLc = primecm.get_confidence_ShoulderL();
  sRc = primecm.get_confidence_ShoulderR();
  nc = primecm.get_confidence_Neck();
  nc = primecm.get_confidence_Head();
  tc = primecm.get_confidence_Torso();
  if( sLc[1]==0 or sRc[1]==0 or nc[1]==0 or tc[1]==0 ) then
    return rpy;
  end

  -- Head to Waist and Shoulder to Shoulder Vectors define the bases
  t2n = primecm.get_position_Head() - primecm.get_position_Torso();
  s2s = primecm.get_position_ShoulderR() - primecm.get_position_ShoulderL();

  -- These should form an orthonormal basis, so normalize
  t2n = t2n / vector.norm(t2n);
  s2s = s2s / vector.norm(s2s);

  -- Find the cross product
  chest = cross(s2s,t2n);

  -- Remap the coordinates
  local u = vector.new({chest[3],chest[1],chest[2]})
  local v = vector.new({s2s[3],s2s[1],s2s[2]})
  local w = vector.new({t2n[3],t2n[1],t2n[2]})

  -- Find the closest Orthonormal Matrix
  M = matrix.transpose( matrix{u,v,w} )
  Rtorso = M*( (matrix.transpose(M)*M)^-1/2 or 1 )

  -- Grab Euler Angles for SJ --http://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation#Roll-Pitch-Yaw_Angles
  local r = math.atan2(Rtorso[3][2],Rtorso[3][3])
  local y = math.atan2(Rtorso[2][1],Rtorso[1][1])
  local p = math.atan2(-1*Rtorso[3][1],math.cos(y)*Rtorso[1][1]+math.sin(y)*Rtorso[2][1])

  -- Clamp them
  r = util.procFunc(r,0,10*math.pi/180)
  p = util.procFunc(p,0,20*math.pi/180)
  y = util.procFunc(y,0,30*math.pi/180)

  rpy = vector.new({r,p,y})

end

function update_height()
  local t2n = primecm.get_position_Head() - primecm.get_position_Torso();
  local heightoffset = primecm.get_position_Torso() - primecm.get_skeleton_torsocenter();
  --	local t2n = primecm.get_position_Head() - primecm.get_position_Waist();
  --	local heightoffset = primecm.get_position_Waist() - primecm.get_skeleton_torsocenter();
  local heightfactor = vector.norm(t2n)
  heightoffset = heightoffset[2] / heightfactor;
  if heightoffset>0 then
    heightoffset = 0;
  else
    -- Add scaling factor to OP
    --heightoffset = .75 * heightoffset
    heightoffset = .5 * heightoffset
  end
  bodyHeight = Config.walk.bodyHeight + heightoffset;
  bodyHeight = math.min( Config.walk.bodyHeight,math.max(Config.stance.bodyHeightSit,bodyHeight) )
end

function update_velocity()
  -- Roll
  local roll0 = 0; -- Roll offset
  local vpr = 2; -- Tuned value
  local r_deadband = 5*math.pi/180;
  -- Pitch
  local pitch0 = -5*math.pi/180; -- Tuned value
  local vpp = -0.5; -- Tuned
  local vpp = -0.1; -- Tuned
  local p_deadband = 10*math.pi/180;
  -- Yaw
  local yaw0 = 0;
  local vpy = .5; -- Tuned value
  local y_deadband = 15*math.pi/180;
  
  
  local vx = rpy[2] - pitch0
  if math.abs(vx)<p_deadband then
    vx = 0;
  end 
  vx = vpp * vx;
--[[
  local vy = rpy[1] - roll0
  if math.abs(vy)<r_deadband then
    vy = 0;
  end 
  vy = vpr * vy;
--]]
  local va = rpy[3] - yaw0
  if math.abs(va)<y_deadband then
    va = 0;
  end 
  va = vpy * va;
 
  --[[
  local vy = rpy[3] - yaw0
  if math.abs(vy)<y_deadband then
    vy = 0;
  end 
  vy = vpy * vy;
  
  local va = rpy[1] - roll0
  if math.abs(va)<r_deadband then
    va = 0;
  end 
  va = vpr * va;
--]]
  velocity = vector.new({vx,vy,va});
  --print(string.format('Vel %.3f, %.3f, %.3f',unpack(velocity)) )
  --print(string.format('RPY %.3f, %.3f, %.3f\n',unpack(rpy)) )
end

function cross(v1,v2)
  v3 = vector.zeros(3);
  v3[1] =   ( (v1[2] * v2[3]) - (v1[3] * v2[2]) )
  v3[2] = - ( (v1[1] * v2[3]) - (v1[3] * v2[1]) )
  v3[3] =   ( (v1[1] * v2[2]) - (v1[2] * v2[1]) )
  return v3; 
end

function round2(num, idp)
  local ret = vector.zeros(#num)
  for i=1,#num do
    ret[i] = tonumber(string.format("%." .. (idp or 0) .. "f", num[i])) 
  end 
  return ret;
end
