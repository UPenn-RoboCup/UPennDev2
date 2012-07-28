module(..., package.seeall);

require 'Config'
require('Body')
require('boxercm')
require('walk')
require('vector')

t0 = 0;
timeout = 5;
punching = false;

-- Start with left arm forward
cur_sign = 1;
sides = {};
sides[1] = 'right'
sides[-1] = 'left'
torsoYaw = cur_sign*45*math.pi/180;

-- Default arm position
qLArm = math.pi/180*vector.new({90,40,-160});
qRArm = math.pi/180*vector.new({90,-40,-160});

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
end

function update()
  t = Body.get_time();

  -- Check if there is a punch activated
  local pL = boxercm.get_body_punchL();
  local pR = boxercm.get_body_punchR();

  if( not punching ) then
    if( pL==1 and pR==0 ) then -- left arm punch
      doPunch('left')
    elseif( pL==0 and pR==1 ) then -- right arm punch
      doPunch('right')
    elseif( pL==1 and pR==1 ) then -- both arm punch (pushaway)
      -- Switch states
      return 'doublepunch'
    else -- No punch
    end
  end

  if( punching ) then
    doPunch();
  end

  -- Always override
  walk.upper_body_override(qLArm, qRArm, {0,0,torsoYaw});

  if( boxercm.get_body_enabled()==0 and not punching ) then
    print('Boxer Disabled!')
    return "disabled";
  end

end

function exit()
end

function doPunch(side)

  if( side ) then
    print("My sides: ",sides[cur_sign],side)
    -- Initialize the stance changes
    if( sides[cur_sign]==side ) then
      print('no switch!')
      footing = false;
    else
      footing = true;
      tStance0 = t;
      tStance1 = tStance0 + walk.tStep;
      yaw0 = cur_sign*45*math.pi/180;
      yaw1 = -1*cur_sign*45*math.pi/180;
      -- Execute the kick motion
      walk.doPunch(side);
    end

    -- Initialize the arm swing
    if( sides[cur_sign]==side ) then
      armMotion = Config.walk.motionDef['jab'..side];
    else
      --armMotion = Config.walk.motionDef['swing'..side];
      armMotion = Config.walk.motionDef['jab'..side];
    end
    tArm0 = t;
    armStage = 1;
    arming = true;

    -- Ensure that we know we are punching
    punching = true;
    return;
  end

  -- Phase the arms for the punch
  if( arming ) then
    local ph_arm = math.max(math.min((t-tArm0)/armMotion[armStage][1],1),0);
    -- Default arm position
    qLArm = ph_arm*vector.new(armMotion[armStage][2])+(1-ph_arm)*qLArm;
    qRArm = ph_arm*vector.new(armMotion[armStage][3])+(1-ph_arm)*qRArm;
    if(ph_arm>=1) then
      tArm0 = tArm0 + armMotion[armStage][1];
      armStage = armStage+1;
    end
    if(armStage>#armMotion) then
      arming = false;
    end
  end

  -- Execute the kick and phase the body yaw
  if( footing ) then
    local ph_yaw = math.max(math.min((t-tStance0)/(tStance1-tStance0),1),0);
    torsoYaw = (1-ph_yaw)*yaw0 + ph_yaw*yaw1;

    -- Done doing the kick swap? done the yaw swap?
    if(walk.walkKickRequest==0 and ph_yaw>=1) then
      footing = false;
    end
  end

  -- Done punch!
  if( not footing and not arming ) then
    print('Done punch!');
    cur_sign = -1*cur_sign;
    punching = false;
  end

end

-- TO switch stance:
--[[
function check_stance()
t=Body.get_time();
uTorsoOffset0 = {uTorsoOffset[1],uTorsoOffset[2],uTorsoOffset[3]};
if stance1==0 then    --Standard stance
uLRFootOffset = vector.new({0,footY,0});
uTorsoOffsetTarget = {-footX,0,0};
tStance0=t;tStance1 = t+tStep;
elseif stance1==1 then    --Left-front stance
uLRFootOffset = vector.new({0.03,footY,0});
uTorsoOffsetTarget = {-footX+0.01,0,-45*math.pi/180};
tStance0=t;tStance1 = t+tStep;
else    --Right-front stance
uLRFootOffset = vector.new({-0.03,footY,0});
uTorsoOffsetTarget = {-footX+0.01,0,45*math.pi/180};
tStance0=t;tStance1 = t+tStep;
end
stance=stance1;
end
--]]
-- TODO: Just blend the bodyRot from -45 to 45 in a matter of one tStep
-- so that there is no jerkiness
-- also, BodyFSM runs just before every Motion update
