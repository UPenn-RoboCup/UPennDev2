module(..., package.seeall);

require 'wcm'
require 'gcm'
require 'util'
playerID = gcm.get_team_player_id();
log_filename = "football_loss.log"

-- Initialize
-- Set of policies is the set of function calls
if( playerID==1 ) then
  policies = {'direct','predict','mirror','go_direct','go_predict'}
  trial = 1;
  local r_script = 'Rscript ./Player/Util/getAction.R '..#policies..' '..trial;
  getAction  = io.popen(r_script);
  policy = tonumber( getAction:read() );
  print( 'Initial tackle policy', policy)


else
  avoid_type = math.floor( math.random() * 4 );
  print('Initial avoid_type: ',avoid_type)
end
io:flush()

-- Field params
maxStep = 0.06;
field_sz = 7;
x_start = 3;


function record_yardage( )
  if( playerID==1 ) then
    local opose = footballcm.get_opponent_pose();
    local yardage = x_start - opose[1];

    -- Clamp: more yards --> 0, less yards --> 1
    local loss = 1 - math.min( math.max(yardage / field_sz,0), 1);
    -- Log data
    -- Open the log file
    local log_f = assert(io.open(log_filename, "a"))
    if(trial==1) then
      local title_str = string.format('Trial Policy Loss\n')
    end
    local log_str = string.format('%d %d %f\n',trial,policy,loss)
    log_f:write( log_str )
    log_f:close()
    
    print('Trial '..trial)
    print('Policy ',policies[policy])
    print('Yardage/Loss:',yardage,loss);
    -- Call Hyunseung's function and set the next policy
    trial = trial + 1;
    local r_script = 'Rscript ./Player/Util/getAction.R '..#policies..' '..trial..' '..loss;
    getAction  = io.popen(r_script);
    nxt_policy = tonumber( getAction:read() );
    if( not nxt_policy or nxt_policy > #policies ) then
      print('Script call: ',r_script)
      print('Next policy out of range! ',nxt_policy)
    else
      policy = nxt_policy;
      print('Next Policy',policies[policy]);
    end


  else
    -- Get the avoider policy
    avoid_type = math.floor( math.random() * 4 );
    print('Next avoid_type: ',avoid_type)
  end

  print();
  io:flush();
  reset_vars();
end

function update( )
  if( playerID==1 ) then
    return getfenv()[policies[policy]]();
  else
    return avoid();
  end
end

function get_dist()
  local opose    = footballcm.get_opponent_pose();
  local gps_pose = wcm.get_robot_gpspose();
  local oppRelative = util.pose_relative(
  opose, gps_pose
  );
  rOppRelative = math.sqrt(oppRelative[1]^2 + oppRelative[2]^2);
  return rOppRelative;
end

-- Randomly choose an avoidance policy
function avoid()
  local ret = {}
  ret.vx = 0;
  ret.vy = 0;
  ret.va = 0;

  if( avoid_type==1 ) then
    -- Go to opposite corner
    local opose    = footballcm.get_opponent_pose();
    local gps_pose = wcm.get_robot_gpspose();


    local oppRelative = util.pose_relative(
    opose, gps_pose
    );

    local goalpose = vector.new({-3.5,2*util.sign(oppRelative[2]),math.pi});

    local goalRelative = util.pose_relative(
    goalpose, gps_pose
    );
    rGoalRelative = math.sqrt(goalRelative[1]^2 + goalRelative[2]^2);

    ret.vx = maxStep * goalRelative[1]/rGoalRelative;
    ret.vy = maxStep * goalRelative[2]/rGoalRelative;
    ret.va = .2 * math.atan2(goalRelative[2], 0 - goalRelative[1]);
  elseif( avoid_type==2 ) then
    -- Avoid in the y direction
    ret = direct();
    ret.vy = ret.vy * -1;
  elseif(avoid_type==3) then
    ret = predict();
    ret.vy = ret.vy * -1;
  else
    ret.vx = maxStep;
    ret.vy = 0;
    ret.va = 0;
  end
  return ret;
end

function direct()
  local opose    = footballcm.get_opponent_pose();
  local gps_pose = wcm.get_robot_gpspose();
  local ret = {};

  local oppRelative = util.pose_relative(
  opose, gps_pose
  );
  rOppRelative = math.sqrt(oppRelative[1]^2 + oppRelative[2]^2);

  ret.vx = maxStep * oppRelative[1]/rOppRelative;
  ret.vy = maxStep * oppRelative[2]/rOppRelative;
  ret.va = .2 * math.atan2(oppRelative[2], 0 - oppRelative[1]);
  return ret;
end

function predict()
  local ret = {}

  -- Get opponent's velocity
  local opose_prev = opose;
  opose = footballcm.get_opponent_pose();
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

function mirror()
  local opose    = footballcm.get_opponent_pose();
  local gps_pose = wcm.get_robot_gpspose()
  local diff = (opose-gps_pose);
  if( math.abs(diff[2])<1 ) then -- within a yar mirrored
    return direct();
  else
    local ret = {}
    ret.vx = 0;
    ret.vy = maxStep * util.sign(diff[2]);
    ret.va = 0;
    return ret;
  end

end

function go_direct()
  ret = {}
  ret.vx = maxStep;
  ret.vy = 0;
  ret.va = 0;
  -- Sprint until closer to the robot
  if( get_dist() < 4 ) then
    return direct()
  else
    return ret;
  end
end

function go_predict()
  ret = {}
  ret.vx = maxStep;
  ret.vy = 0;
  ret.va = 0;
  -- Sprint until closer to the robot
  if( get_dist() < 4 ) then
    return predict()
  else
    return ret;
  end
end

function reset_vars()
  opose = footballcm.get_opponent_pose();
  t_opp = Body.get_time();
  o_vx = 0;
  o_vy = 0;
end
-- Call this initially
reset_vars();
