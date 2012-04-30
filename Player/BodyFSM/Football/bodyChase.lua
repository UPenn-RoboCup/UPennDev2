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

  Speak.talk('chasing time!')

  t0 = Body.get_time();
end

function update()
  local t = Body.get_time();

  -- get opponent position
  local opose = wcm.get_opponent_pose();
  local gps_pose = wcm.get_robot_gpspose();
  --print('Oponent pose:',opose);
  --[[
  local opponent = {};
  opponent.x = opose[1];
  opponent.y = opose[2];
  opponent.a = opose[3];
  --]]
  local va, vx, vy = 0,0,0;

  oppRelative = util.pose_relative(
  opose, gps_pose
  );
  rOppRelative = math.sqrt(oppRelative[1]^2 + oppRelative[2]^2);

  vx = maxStep * oppRelative[1]/rOppRelative;
  vy = maxStep * oppRelative[2]/rOppRelative;
  va = .2 * oppRelative[3];
  walk.set_velocity(vx, vy, va);

  if (t - t0 > timeout) then
    print('opponent distance: ',rOppRelative)
    return "timeout";
  end

--  print('opponent distance: ',rOppRelative)

  if (rOppRelative < rClose) then
    return "close";
  end
end

function exit()
end

