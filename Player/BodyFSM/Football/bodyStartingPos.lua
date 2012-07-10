module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('wcm')
require 'gcm'

t0 = 0;
timeout = 15;
maxStep = 0.06;
thAlign = 10*math.pi/180;
rClose = 0.2;
playerID = gcm.get_team_player_id();
if( playerID==1 ) then
  starting_gps_pose = vector.new({ -3,0, 0});
else
  starting_gps_pose = vector.new({ 3, 0, math.pi});
end
gps_pose = wcm.get_robot_gpspose();

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  gps_pose = wcm.get_robot_gpspose();
  wcm.set_agent_ready(0);
end

function update()

  local t = Body.get_time();
  gps_pose = wcm.get_robot_gpspose();

  local va, vx, vy = 0,0,0;

  startRelative = util.pose_relative( 
  starting_gps_pose, gps_pose
  );
  rStartRelative = math.sqrt(startRelative[1]^2 + startRelative[2]^2);

  vx = maxStep * startRelative[1]/rStartRelative;
  vy = maxStep * startRelative[2]/rStartRelative;
  --va = .2 * math.atan2(startRelative[2], startRelative[1])
  va = .2 * startRelative[3];
  walk.set_velocity(vx, vy, va);

  if ((t - t0 > 1.0) and (rStartRelative < rClose)) then
    return 'done';
  end

  if (t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
  walk.set_velocity(0,0,0);
end

