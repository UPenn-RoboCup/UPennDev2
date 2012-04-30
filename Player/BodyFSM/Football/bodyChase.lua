module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')

t0 = 0;
timeout = 20.0;
-- maximum walk velocity
maxStep = 0.04;
-- opponent distance threshold
rClose = 0.35;

-- opponent detection timeout
tLost = 3.0;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();
end

function update()
  local t = Body.get_time();

  -- get opponent position
  local opose = wcm.get_opponent_pose();
  print('Oponent pose:',opose);
  local opponent = {};
  opponent.x = opose[1];
  opponent.y = opose[2];
  opponent.a = opose[3];

  opponentR = math.sqrt(opponent.x^2 + opponent.y^2);

  -- calculate walk velocity based on opponent position
  vStep = vector.new({0,0,0});
  vStep[1] = .6*opponent.x;
  vStep[2] = .75*opponent.y;
  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;

  opponentA = math.atan2(opponent.y, opponent.x+0.10);
  vStep[3] = 0.75*opponentA;
  walk.set_velocity(vStep[1],vStep[2],vStep[3]);

  if (t - opponent.t > tLost) then
    return "opponentLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end
  if (opponentR < rClose) then
    return "opponentClose";
  end
end

function exit()
end

