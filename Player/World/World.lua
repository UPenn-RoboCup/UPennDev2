module(..., package.seeall);

require('PoseFilter');
require('Filter2D');
require('Body');
require('vector');
require('util');
require('wcm')
require('vcm');
require('gcm');
require('mcm');

-- intialize sound localization if needed
useSoundLocalization = Config.world.enable_sound_localization or 0;
if (useSoundLocalization > 0) then
  require('SoundFilter');
end

--SJ: Velocity filter is always on
--We can toggle whether to use velocity to update ball position estimate
--In Filter2D.lua

mod_angle = util.mod_angle;

--require('Velocity');	

--Are we using same colored goals?
use_same_colored_goal = Config.world.use_same_colored_goal or 0;
--Use ground truth pose and ball information for webots?
use_gps_only = Config.use_gps_only or 0;

ballFilter = Filter2D.new();
ball = {};
ball.t = 0;  --Detection time
ball.x = 1.0;
ball.y = 0;
ball.vx = 0;
ball.vy = 0;

pose = {};
pose.x = 0;
pose.y = 0;
pose.a = 0;
pose.tGoal = 0; --Goal detection time

uOdometry0 = vector.new({0, 0, 0});
count = 0;
cResample = Config.world.cResample; 

playerID = Config.game.playerID;

odomScale = Config.walk.odomScale;
wcm.set_robot_odomScale(odomScale);

--SJ: they are for IMU based navigation
imuYaw = Config.world.imuYaw or 0;
yaw0 =0;

--Track gcm state
gameState = 0;

function init_particles()
  if use_same_colored_goal>0 then
    goalDefend=get_goal_defend();
    PoseFilter.initialize_unified(
      vector.new({goalDefend[1]/2,-2,math.pi/2}),
      vector.new({goalDefend[1]/2,2,-math.pi/2}));
  else
    PoseFilter.initialize();    
  end
end

function allLessThanTenth(table)
  for k,v in pairs(table) do
    if v >= .1 then
      return false
    end
  end
  return true
end

function allZeros(table)
  for k,v in pairs(table) do
    if v~=0 then
      return false
    end
  end
  return true
end

function entry()
  count = 0;
  init_particles();
  --Velocity.entry();
end

function update_odometry()

  --odomScale = Config.walk.odomScale;
  odomScale = wcm.get_robot_odomScale();
  count = count + 1;
  uOdometry, uOdometry0 = mcm.get_odometry(uOdometry0);

  uOdometry[1] = odomScale[1]*uOdometry[1];
  uOdometry[2] = odomScale[2]*uOdometry[2];
  uOdometry[3] = odomScale[3]*uOdometry[3];

  --Gyro integration based IMU
  if imuYaw==1 then
    yaw = Body.get_sensor_imuAngle(3);
    uOdometry[3] = yaw-yaw0;
    yaw0 = yaw;
    --print("Body yaw:",yaw*180/math.pi, " Pose yaw ",pose.a*180/math.pi)
  end

  ballFilter:odometry(uOdometry[1], uOdometry[2], uOdometry[3]);
  PoseFilter.odometry(uOdometry[1], uOdometry[2], uOdometry[3]);
  if (useSoundLocalization > 0) then
    SoundFilter.odometry(uOdometry[1], uOdometry[2], uOdometry[3]);
    SoundFilter.update();
  end
end


function update_pos()
  -- update localization without vision (for odometry testing)
  if count % cResample == 0 then
    PoseFilter.resample();
  end

  pose.x,pose.y,pose.a = PoseFilter.get_pose();
  update_shm();
end


function update_vision()

  --update ground truth
  if Body.gps_enable then
    gps_pose0=Body.get_sensor_gps();
    --GPS is attached at torso, so we should discount body offset
    uBodyOffset = mcm.get_walk_bodyOffset();
    gps_pose = util.pose_global(-uBodyOffset,gps_pose0);

    gps_pose_xya={}
    gps_pose_xya.x=gps_pose[1];
    gps_pose_xya.y=gps_pose[2];
    gps_pose_xya.a=gps_pose[3];
    gps_attackBearing = get_attack_bearing_pose(gps_pose_xya);

    wcm.set_robot_gpspose(gps_pose);
    wcm.set_robot_gps_attackbearing(gps_attackBearing);
  else
    wcm.set_robot_gpspose({pose.x,pose.y,pose.a});
    wcm.set_robot_gps_attackbearing(get_attack_bearing());
  end

  --We may use ground truth data only (for behavior testing)
  if use_gps_only>0 then
    --Use GPS pose instead of using particle filter
    pose.x,pose.y,pose.a=gps_pose[1],gps_pose[2],gps_pose[3];
    --Use GPS ball pose instead of ball filter
    ballGlobal=wcm.get_robot_gps_ball();    
    ballLocal = util.pose_relative(ballGlobal,gps_pose);
    ball.x, ball.y = ballLocal[1],ballLocal[2];

    if vcm.get_ball_detect()==1 then
      ball.t = Body.get_time();
    end
    update_shm();
    return;
  end

  -- resample?
  if count % cResample == 0 then
    PoseFilter.resample();
    PoseFilter.add_noise();
  end

  -- Reset heading if robot is down
  if (mcm.get_walk_isFallDown() == 1) then
    PoseFilter.reset_heading();

    if (useSoundLocalization > 0) then
      SoundFilter.reset();
    end
  end

  gameState = gcm.get_game_state();
  if (gameState == 0) then
    init_particles();

    if (useSoundLocalization > 0) then
      SoundFilter.reset();
    end
  end

  --If robot was in penalty and game switches to set, initialize particles
  --for manual placement
  if wcm.get_robot_penalty() == 1 and gcm.get_game_state() == 2 then
    PoseFilter.initialize_manual_placement()
  elseif gcm.in_penalty() then
    init_particles()
  end

  -- Penalized?
  if gcm.in_penalty() then
    wcm.set_robot_penalty(1);
  else
    wcm.set_robot_penalty(0);
  end

  fsrRight = Body.get_sensor_fsrRight()
  fsrLeft = Body.get_sensor_fsrLeft()

  --reset particle to face opposite goal when getting manual placement on set
  if gcm.get_game_state() ==2 then
    if (not allZeros(fsrRight)) and (not allZeros(fsrLeft)) then --Do not do this if sensor is broken
      if allLessThanTenth(fsrRight) and allLessThanTenth(fsrLeft) then
        PoseFilter.initialize_manual_placement();
      end
    end
  end
    
  -- ball
  if (vcm.get_ball_detect() == 1) then
    ball.t = Body.get_time();
    local v = vcm.get_ball_v();
    local dr = vcm.get_ball_dr();
    local da = vcm.get_ball_da();
    ballFilter:observation_xy(v[1], v[2], dr, da);
    Body.set_indicator_ball({1,0,0});

    --[[
    -- Update the velocity
    Velocity.update(v[1],v[2]);
    ball.vx, ball.vy, dodge  = Velocity.getVelocity();
    local speed = math.sqrt(ball.vx^2 + ball.vy^2);
    local stillTime = mcm.get_walk_stillTime();
    if( stillTime > 1.5 ) then 
--      print('Speed: '..speed..', Vel: ('..ball.vx..', '..ball.vy..') Still Time: '..stillTime);
    end
    --]]
  else
    --Velocity.update_noball();--notify that ball is missing
    Body.set_indicator_ball({0,0,0});
  end

  -- TODO: handle goal detections more generically
  
  if vcm.get_goal_detect() == 1 then
    pose.tGoal = Body.get_time();
    local color = vcm.get_goal_color();
    local goalType = vcm.get_goal_type();
    local v1 = vcm.get_goal_v1();
    local v2 = vcm.get_goal_v2();
    local v = {v1, v2};

    if (use_same_colored_goal > 0) then
      -- resolve attacking/defending goal using the goalie sound localization
      --  0 - unknown
      -- -1 - defending
      -- +1 - attacking
      local attackingOrDefending = 0;
      if (useSoundLocalization > 0) then
        attackingOrDefending = SoundFilter.resolve_goal_detection(goalType, v); 
      end
      
      if (attackingOrDefending == 1) then
        -- attacking goal
        if (gcm.get_team_color() == 1) then
          -- we are the red team, shooting on cyan goal
          if (goalType == 0) then
            PoseFilter.post_cyan_unknown(v);
          elseif(goalType == 1) then
            PoseFilter.post_cyan_left(v);
          elseif(goalType == 2) then
            PoseFilter.post_cyan_right(v);
          elseif(goalType == 3) then
            PoseFilter.goal_cyan(v);
          end
          -- indicator
          Body.set_indicator_goal({0,0,1});
        else
          if (goalType == 0) then
            PoseFilter.post_yellow_unknown(v);
          elseif(goalType == 1) then
            PoseFilter.post_yellow_left(v);
          elseif(goalType == 2) then
            PoseFilter.post_yellow_right(v);
          elseif(goalType == 3) then
            PoseFilter.goal_yellow(v);
          end
          -- indicator
          Body.set_indicator_goal({1,1,0});
        end
      
      elseif (attackingOrDefending == -1) then
        -- defending goal
        if (gcm.get_team_color() == 1) then
          -- we are the red team, defending the yellow goal
          if (goalType == 0) then
            PoseFilter.post_yellow_unknown(v);
          elseif(goalType == 1) then
            PoseFilter.post_yellow_left(v);
          elseif(goalType == 2) then
            PoseFilter.post_yellow_right(v);
          elseif(goalType == 3) then
            PoseFilter.goal_yellow(v);
          end
          -- indicator
          Body.set_indicator_goal({1,1,0});
        else
          if (goalType == 0) then
            PoseFilter.post_cyan_unknown(v);
          elseif(goalType == 1) then
            PoseFilter.post_cyan_left(v);
          elseif(goalType == 2) then
            PoseFilter.post_cyan_right(v);
          elseif(goalType == 3) then
            PoseFilter.goal_cyan(v);
          end
          -- indicator
          Body.set_indicator_goal({0,0,1});
        end

      else
        -- we dont know which goal it is
        if (goalType == 0) then
          PoseFilter.post_unified_unknown(v);
          Body.set_indicator_goal({1,1,0});
        elseif(goalType == 1) then
          PoseFilter.post_unified_left(v);
          Body.set_indicator_goal({1,1,0});
        elseif(goalType == 2) then
          PoseFilter.post_unified_right(v);
          Body.set_indicator_goal({1,1,0});
        elseif(goalType == 3) then
          PoseFilter.goal_unified(v);
          Body.set_indicator_goal({0,0,1});
        end
      end

    else
      --Goal observation with colors
      if color == Config.color.yellow then
        if (goalType == 0) then
          PoseFilter.post_yellow_unknown(v);
        elseif(goalType == 1) then
          PoseFilter.post_yellow_left(v);
        elseif(goalType == 2) then
          PoseFilter.post_yellow_right(v);
        elseif(goalType == 3) then
          PoseFilter.goal_yellow(v);
        end
        -- indicator
        Body.set_indicator_goal({1,1,0});
      elseif color == Config.color.cyan then
        if (goalType == 0) then
          PoseFilter.post_cyan_unknown(v);
        elseif(goalType == 1) then
          PoseFilter.post_cyan_left(v);
        elseif(goalType == 2) then
          PoseFilter.post_cyan_right(v);
        elseif(goalType == 3) then
          PoseFilter.goal_cyan(v);
        end
        -- indicator
        Body.set_indicator_goal({0,0,1});
      end
    end
  else
    -- indicator
    Body.set_indicator_goal({0,0,0});
  end

  -- line update
  if vcm.get_line_detect() == 1 then
    local v = vcm.get_line_v();
    local a = vcm.get_line_angle();

    PoseFilter.line(v, a);--use longest line in the view
  end

  if vcm.get_corner_detect() == 1 then
    local v=vcm.get_corner_v();
    PoseFilter.corner(v);
  end

  if vcm.get_landmark_detect() == 1 then
    local color = vcm.get_landmark_color();
    local v = vcm.get_landmark_v();
    if color == Config.color.yellow then
        PoseFilter.landmark_yellow(v);
    else
        PoseFilter.landmark_cyan(v);
    end
  end

  ball.x, ball.y = ballFilter:get_xy();
  pose.x,pose.y,pose.a = PoseFilter.get_pose();

  update_shm();
end

function update_shm()
  -- update shm values

  --print(string.format( 
  wcm.set_robot_pose({pose.x, pose.y, pose.a});
  wcm.set_robot_time(Body.get_time());

  wcm.set_ball_x(ball.x);
  wcm.set_ball_y(ball.y);
  wcm.set_ball_t(ball.t);
  wcm.set_ball_velx(ball.vx);
  wcm.set_ball_vely(ball.vy);

  wcm.set_goal_t(pose.tGoal);
  wcm.set_goal_attack(get_goal_attack());
  wcm.set_goal_defend(get_goal_defend());
  wcm.set_goal_attack_bearing(get_attack_bearing());
  wcm.set_goal_attack_angle(get_attack_angle());
  wcm.set_goal_defend_angle(get_defend_angle());

  wcm.set_goal_attack_post1(get_attack_posts()[1]);
  wcm.set_goal_attack_post2(get_attack_posts()[2]);

  wcm.set_robot_is_fall_down(mcm.get_walk_isFallDown());
  --Particle information
  wcm.set_particle_x(PoseFilter.xp);
  wcm.set_particle_y(PoseFilter.yp);
  wcm.set_particle_a(PoseFilter.ap);
  wcm.set_particle_w(PoseFilter.wp);

end

function exit()
end


function get_ball()
  return ball;
end

function get_pose()
  return pose;
end

function zero_pose()
  PoseFilter.zero_pose();
end

function get_attack_bearing()
  return get_attack_bearing_pose(pose);
end

--Get attack bearing from pose0
function get_attack_bearing_pose(pose0)
  if gcm.get_team_color() == 1 then
    -- red attacks cyan goal
    postAttack = PoseFilter.postCyan;
  else
    -- blue attack yellow goal
    postAttack = PoseFilter.postYellow;
  end
  -- make sure not to shoot back towards defensive goal:
  local xPose = math.min(math.max(pose0.x, -0.99*PoseFilter.xLineBoundary),
                          0.99*PoseFilter.xLineBoundary);
  local yPose = pose0.y;
  local aPost = {}
  aPost[1] = math.atan2(postAttack[1][2]-yPose, postAttack[1][1]-xPose);
  aPost[2] = math.atan2(postAttack[2][2]-yPose, postAttack[2][1]-xPose);
  local daPost = math.abs(PoseFilter.mod_angle(aPost[1]-aPost[2]));
  attackHeading = aPost[2] + .5*daPost;
  attackBearing = PoseFilter.mod_angle(attackHeading - pose0.a);

  return attackBearing, daPost;
end

function get_goal_attack()
  if gcm.get_team_color() == 1 then
    -- red attacks cyan goal
    return {PoseFilter.postCyan[1][1], 0, 0};
  else
    -- blue attack yellow goal
    return {PoseFilter.postYellow[1][1], 0, 0};
  end
end

function get_goal_defend()
  if gcm.get_team_color() == 1 then
    -- red defends yellow goal
    return {PoseFilter.postYellow[1][1], 0, 0};
  else
    -- blue defends cyan goal
    return {PoseFilter.postCyan[1][1], 0, 0};
  end
end

function get_attack_posts()
  if gcm.get_team_color() == 1 then
    return Config.world.postCyan;
  else
    return Config.world.postYellow;
  end
end

function get_attack_angle()
  goalAttack = get_goal_attack();

  dx = goalAttack[1] - pose.x;
  dy = goalAttack[2] - pose.y;
  return mod_angle(math.atan2(dy, dx) - pose.a);
end

function get_defend_angle()
  goalDefend = get_goal_defend();

  dx = goalDefend[1] - pose.x;
  dy = goalDefend[2] - pose.y;
  return mod_angle(math.atan2(dy, dx) - pose.a);
end

function get_team_color()
  return gcm.get_team_color();
end

function pose_global(pRelative, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  return vector.new{pose[1] + ca*pRelative[1] - sa*pRelative[2],
                    pose[2] + sa*pRelative[1] + ca*pRelative[2],
                    pose[3] + pRelative[3]};
end

function pose_relative(pGlobal, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  local px = pGlobal[1]-pose[1];
  local py = pGlobal[2]-pose[2];
  local pa = pGlobal[3]-pose[3];
  return vector.new{ca*px + sa*py, -sa*px + ca*py, mod_angle(pa)};
end


