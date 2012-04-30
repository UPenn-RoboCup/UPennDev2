module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('wcm')
require('Config')
-- 2 is master
t0 = 0;
timeout =Config.BodyFSM.orbit.timeout;
maxStep =Config.BodyFSM.orbit.maxStep;
thAlign = Config.BodyFSM.orbit.thAlign;
playerID = Config.game.playerID;
starting_gps_pose = { -1*util.sign(playerID-1),0};
gps_pose = wcm.get_robot_gpspose();

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  gps_pose = wcm.get_robot_gpspose();
end

function update()

  local t = Body.get_time();
  gps_pose = wcm.get_robot_gpspose();

  local va, vx, vy = 0,0,0;

  startRelative = util.pose_relative( 
  centerPosition, 
  {gps_pose.x, gps_pose.y, gps_pose.a}
  );
  rStartRelative = math.sqrt(startRelative[1]^2 + startRelative    [2]^2);

  vx = maxStep * startRelative[1]/rStartRelative;
  vy = maxStep * startRelative[2]/rStartRelative;
  va = .2 * startRelative[3];
  walk.set_velocity(vx, vy, va);

  if ((t - t0 > 2.0) and (rStartRelative < rClose)) then
    return 'done';
  end

  if (t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
end

