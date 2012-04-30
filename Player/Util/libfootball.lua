module(..., package.seeall);

require 'wcm'
require 'gcm'
require 'util'
playerID = gcm.get_team_player_id();

policy = 2;
trial = 1;
maxStep = 0.06;

-- Set of policies is the set of function calls
policies = {'direct','predict'}

function record_yardage( )
  if( playerID==1 ) then
    local opose = wcm.get_opponent_pose();
    local yardage = math.abs( opose[1] - 2 );
    Speak.talk('Gained '..yardage..' yards')
    print('Trial '..trial)
    print('Policy ',policies[policy])
    print('Yardage gained:',yardage);
    -- TODO: Call Hyunseung's function and set the next policy
    trial = trial + 1;
    reset_vars();
  end
end

function update( )
  if( playerID==1 ) then
    return getfenv()[policies[policy]]();
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
  ret.vx = maxStep;
  ret.vy = 0;
  ret.va = 0;
  return ret;
end

function direct()
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

function predict()
  local ret = {}

  -- Get opponent's velocity
  local opose_prev = opose;
  opose = wcm.get_opponent_pose();
  local opose_diff = opose - opose_prev;
  local t_prev = t_opp;
  t_opp = Body.get_time();
  local t_diff = t_opp - t_prev;
  if( t_diff < 1 ) then
    o_vx = opose_diff[1] / t_diff;
    o_vy = opose_diff[2] / t_diff;
    -- Filter
    local beta = .6;
    local o_vx_prev = o_vx;
    local o_vy_prev = o_vy;
    o_vx = beta * o_vx + (1-beta)*o_vx_prev;
    o_vy = beta * o_vy + (1-beta)*o_vy_prev;
  else
    o_vx = 0;
    o_vy = 0;
  end

  -- Predict position in 2 seconds
  local t_future = 2;
  opose_pred = opose + t_future * vector.new({o_vx,o_vy,0});
  local gps_pose = wcm.get_robot_gpspose();

  -- Now regular direct
  local oppRelative = util.pose_relative(
  opose_pred, gps_pose
  );
  rOppRelative = math.sqrt(oppRelative[1]^2 + oppRelative[2]^2);

  ret.vx = maxStep * oppRelative[1]/rOppRelative;
  ret.vy = maxStep * oppRelative[2]/rOppRelative;
  ret.va = math.atan2(oppRelative[2], 0 - oppRelative[1]);

  return ret;
end

function reset_vars()
  opose = wcm.get_opponent_pose();
  t_opp = Body.get_time();
  o_vx = 0;
  o_vy = 0;
end
-- Call this initially
reset_vars();
