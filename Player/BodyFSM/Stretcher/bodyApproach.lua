module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')

t0 = 0;
timeout = 10.0;

-- maximum walk velocity
maxStep = 0.025;

-- stretcher detection timeout
tLost = 3.0;

-- kick threshold
xKick = 0.16;
xTarget = 0.14;
yKickMin = 0.01;
yKickMax = 0.05;
yTarget0 = 0.04;

-- maximum stretcher distance threshold
rFar = 0.45;

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
  stretcher = wcm.get_stretcher();
  yTarget= util.sign(stretcher.y) * yTarget0;
  Speak2.talk('Time to approach!');
end

function update()
  local t = Body.get_time();

  -- get stretcher position
  stretcher = wcm.get_stretcher();
  stretcherR = math.sqrt(stretcher.x^2 + stretcher.y^2);

  -- calculate walk velocity based on stretcher position
  vStep = vector.new({0,0,0});
  vStep[1] = .6*(stretcher.x - xTarget);
  vStep[2] = .75*(stretcher.y - yTarget);
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  --[[
  stretcherA = math.atan2(stretcher.y - math.max(math.min(stretcher.y, 0.05), -0.05), stretcher.x+0.10);
  vStep[3] = 0.5*stretcherA;
  --]]
  vStep[3]=0; -- Do not rotate into position...
  -- TODO: Find the correct approaching rotation
  -- TODO: Use SJ's approach code
  walk.set_velocity(vStep[1],vStep[2],vStep[3]);

  if (t - stretcher.t > tLost) then
    return "stretcherLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end
  if (stretcherR > rFar) then
    return "stretcherFar";
  end

  if ((stretcher.x < xKick) and (math.abs(stretcher.y) < yKickMax) and
    (math.abs(stretcher.y) > yKickMin)) then
    Team.setTask( 1 ); -- Wait for pickup
    return "pickup";
  end

end

function exit()
end

