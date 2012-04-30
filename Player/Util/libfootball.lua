module(..., package.seeall);

require 'wcm'
require 'gcm'
playerID = gcm.get_team_player_id();

policy = 1;
rOppRelative = 999;

-- Set of policies is the set of function calls
policies = {'direct','predict'}

function set_policy( new_policy )
  policy = new_policy;
end

function update( )
  if( playerID==1 ) then
--    print('Calling ',policies[policy])
--    return _G[policies[policy]]();
    return direct();
  else
    return avoid();
  end
end

function get_dist()
  local opose    = wcm.get_opponent_pose();
  local gps_pose = wcm.get_robot_gpspose();
  local oppRelative = util.pose_relative(
  opose, gps_pose
  );
  rOppRelative = math.sqrt(oppRelative[1]^2 + oppRelative[2]^2);
  return rOppRelative;
end

function avoid()
  local ret = {}
  ret.vx = 0;
  ret.vy = 0;
  ret.va = 0;
  return ret;
end

function direct()
  local maxStep = 0.06;
  local opose    = wcm.get_opponent_pose();
  local gps_pose = wcm.get_robot_gpspose();
  local ret = {};

  local oppRelative = util.pose_relative(
  opose, gps_pose
  );
  rOppRelative = math.sqrt(oppRelative[1]^2 + oppRelative[2]^2);

  ret.vx = maxStep * oppRelative[1]/rOppRelative;
  ret.vy = maxStep * oppRelative[2]/rOppRelative;
  ret.va = math.atan2(oppRelative[2], 0 - oppRelative[1]);
--  ret.va = .2 * oppRelative[3];

  return ret;

end
