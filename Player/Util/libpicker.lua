module(..., package.seeall);
require('Config');
require 'Kinematics'
require('Transform');
require('vector');
require('vcm');
require 'util'

-- Enable Webots specific
if (string.find(Config.platform.name,'Webots')) then
  webots = true;
end

-- CoM is 0.0765-0.0505 = 0.026 below shoulder
-- CoM is 0.0962 above hip
-- Why from CoM???  Maybe the tilt is made about the CoM and bodyHeight is at CoM
shoulderZ = 0.026;
footX     = Config.walk.footX;

upperArmLength = .060; -- OP, spec
lowerArmLength = .129; -- OP, spec

function update()
  --bodyHeight = vcm.get_camera_bodyHeight();
  --bodyTilt   = vcm.get_camera_bodyTilt();
  bodyHeight = mcm.get_walk_bodyHeight();
  bodyTilt   = -1*mcm.get_walk_rpy()[2];

  -- Run the forward kinematics
  --local qLArm = mcm.get_walk_qLArm()
  --local qRArm = mcm.get_walk_qRArm()
  qLArm = vector.new( Body.get_larm_position() )
  qRArm = vector.new( Body.get_rarm_position() )

  tLArm = Kinematics.forward_larm(qLArm);
  pLArm = vector.new({tLArm[1][4],tLArm[2][4],tLArm[3][4]})
  tRArm = Kinematics.forward_rarm(qRArm);
  pRArm = vector.new({tRArm[1][4],tRArm[2][4],tRArm[3][4]})

  -- Get the transform to the shoulder
  tShoulder = Transform.trans(-footX,0,bodyHeight); 
  tShoulder = tShoulder*Transform.rotY(bodyTilt);
  tShoulder = tShoulder*Transform.trans(0,0,shoulderZ);

  -- Update end effector positions
  vShoulderL = vector.new({pLArm[1],pLArm[2],pLArm[3],1});
  vShoulderL = tShoulder * vShoulderL;
  vShoulderL = vShoulderL / vShoulderL[4];
  vShoulderL = vector.slice(vShoulderL,1,3)
  vcm.set_pick_pLArm( vShoulderL );

  vShoulderR = vector.new({pRArm[1],pRArm[2],pRArm[3],1});
  vShoulderR = tShoulder * vShoulderR;
  vShoulderR = vShoulderR / vShoulderR[4];
  vShoulderR = vector.slice(vShoulderR,1,3)
  vcm.set_pick_pRArm( vShoulderR );


  -- Debugging messages
  vcm.add_debug_message(string.format(
  "libpicker:\n bodyHeight %.2f bodyTilt %d\n",
  bodyHeight, bodyTilt*180/math.pi
  ));

end

function provide_assistance_path(assistance_level)
  if( not init_path ) then
    -- Get diff from current arm positions
    -- to current object's pickup points
    local pickMajors = {}
    pickMajors[1] = vector.slice(vcm.get_pick_majorgrasps1(),1,2);
    pickMajors[1][3] = 0;
    pickMajors[2] = vector.slice(vcm.get_pick_majorgrasps1(),3,4);
    pickMajors[2][3] = 0;

    -- Left hand picks 1 (default)
    goofy = false;
    if(pickMajors[1][2]<pickMajors[2][2]) then	-- Right hand picks 1
      goofy = true
    end

    local diffLhand = vShoulderL - pickMajors[1];
    if( goofy ) then
      diffLhand[2] = vShoulderL - pickMajors[2];
    end

    local diffRhand = vShoulderR - pickMajors[2];
    if( goofy ) then
      diffRhand[2] = vShoulderR - pickMajors[1];
    end

    -- Done determining path
    init_path = true;
    return;
  end

  -- Guide hand along this path

end

-- Potential to the centroid
function provide_assistance(assistance_level)

  -- Desired change from the user
  local desiredL = vector.new({0,0,0});
  local desiredR = vector.new({0,0,0});
  local magnitudeL = 0;
  local magnitudeR = 0;

  -- If using the kinect, then get just the raw 
  -- position (retargeted raw)
  local kind = pickercm.get_device_kind();
  if kind==1 then -- Kinect
    desiredL = pickercm.get_desired_pLArm();
    desiredR = pickercm.get_desired_pRArm();
    ddesiredL = desiredL - pLArm;
    ddesiredR = desiredR - pRArm;
  else -- XBOX360 controller
    ddesiredL = pickercm.get_desired_dpLArm()
    ddesiredR = pickercm.get_desired_dpRArm()
  end

  magnitudeL = vector.norm( ddesiredL )
  magnitudeR = vector.norm( ddesiredR )

  --print('magnum',magnitudeR,unpack(ddesiredR))

  if assistance_level==1 then
    magnitudeL = 0.05;
    magnitudeR = 0.05;
  end

  -- Centroid
  -- Body Frame
  local centroid = vcm.get_pick_v1();
  centroid = vector.slice(centroid,1,2)
  centroid[3] = 0.075;
  centroid[4] = 1;
  --print( string.format('cp %.3f %.3f %.3f %.3f', unpack(centroid)) );
  -- Arm Frame
  centroid = Transform.inv(tShoulder) * centroid
  --print( string.format('cp %.3f %.3f %.3f %.3f', unpack(centroid)) );
  centroid = vector.slice(centroid,1,3)
  -- Previous pos to centroid is expected
  -- Arm Frame
  local dexpectedL = centroid - pLArm;
  local dexpectedR = centroid - pRArm;
  local rBlobL = vector.norm( dexpectedL );
  local rBlobR = vector.norm( dexpectedR );
  dexpectedL = dexpectedL / vector.norm(dexpectedL);
  dexpectedR = dexpectedR / vector.norm(dexpectedR);
  -- Normalize to adjust for magnitude of movement
  dexpectedL = dexpectedL * magnitudeL;
  dexpectedR = dexpectedR * magnitudeR;

  -- Get the desired and expected endpoints
  -- Arm Frame
  --print( string.format('d0 %.3f %.3f %.3f', unpack(ddesiredL0)) );
  --print( string.format('dL %.3f %.3f %.3f', unpack(ddesiredL)) );
  --print( string.format('pL %.3f %.3f %.3f', unpack(pLArm)) );
  local expectedL = pLArm + dexpectedL;
  local expectedR = pRArm + dexpectedR;
  local desiredL = pLArm + ddesiredL;
  local desiredR = pRArm + ddesiredR;
  --print( string.format('sL %.3f %.3f %.3f', unpack(desiredL)) );
  --print( string.format('eL %.3f %.3f %.3f\n', unpack(expectedL)) );

  -- Mix these vectors linearly
  -- Body frame for the assistance
  -- The assistance level should change based upon how close to the object we are
  local agg_factor = 1; -- How aggresive to help
  local alphaL = math.min(agg_factor/(1+rBlobL^2),1)
  local alphaR = math.min(agg_factor/(1+rBlobR^2),1)
  -- Fully autonomous
  if assistance_level == 1 then
    alphaL = 1;
    alphaR = 1;
  end
  if assistance_level == 0 then
    alphaL = 0;
    alphaR = 0;
  end
  
  local old_school = true;
  if old_school then
    alphaL = assistance_level;
    alphaR = assistance_level;
  end

  local assistL = (1-alphaL)*desiredL + alphaL*expectedL;
  local assistR = (1-alphaR)*desiredR + alphaR*expectedR;
  --print( string.format('aL %.3f %.3f %.3f\n', unpack(desiredL)) );
  
  -- Grab the Inverse Kinematics for the assisted position
  --print( string.format('qL %.3f %.3f %.3f', unpack(qLArm)) );
  local now_qLArm = vector.new( Kinematics.inverse_larm(assistL) )
  local now_qRArm = vector.new( Kinematics.inverse_rarm(assistR) )
  --print( string.format('qL %.3f %.3f %.3f\n', unpack(now_qLArm)) );
  --print( string.format('qL %.3f %.3f %.3f\n', unpack(now_qLArm-qLArm)) );

  -- Apply the assistance
  --if magnitudeL>0.001 then
    --mcm.set_walk_qLArm( round2(now_qLArm,3) )
    mcm.set_walk_qLArm( now_qLArm )
  --end
  --if magnitudeR>0.001 then
    --mcm.set_walk_qRArm( round2(now_qRArm,3) )
    mcm.set_walk_qRArm( now_qRArm )
  --end
  
end

function round2(num, idp)
  idp = idp or 3;
  local ret = vector.zeros(#num)
  for i=1,#num do
    ret[i] = tonumber(string.format("%." .. (idp or 0) .. "f", num[i]))
  end
  return ret;
end
