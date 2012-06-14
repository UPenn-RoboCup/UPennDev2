module(..., package.seeall);

require('Body')
require('walk')
require('util')
require('vector')
require('Config')
require('wcm')
require('gcm')
require('Team')
require('ocm')

t0 = 0;

maxStep = Config.fsm.bodyReady.maxStep;
rClose = Config.fsm.bodyReady.thClose[1];
thClose = Config.fsm.bodyReady.thClose[2];

--Init position for our kickoff
initPosition1 = Config.world.initPosition1;
--Init position for opponents' kickoff
initPosition2 = Config.world.initPosition2;

-- don't start moving right away
tstart = Config.fsm.bodyReady.tStart or 5.0;

phase=0; --0 for wait, 1 for approach, 2 for turn, 3 for end

function entry()
  print(_NAME.." entry");
  phase=0;

  t0 = Body.get_time();
  Motion.event('standup')
end

function getHomePose()
  role=gcm.get_team_role();
  --Now role-based positioning
  goal_defend=wcm.get_goal_defend();
  --role starts with 0
  if gcm.get_game_kickoff() == 1 then
    home=vector.new(initPosition1[role+1]);
  else
    home=vector.new(initPosition2[role+1]);
  end

  --Goalie moves differently
  if role==0 and phase==1 then 
    home=home*0.7;
  end;

  home=home*util.sign(goal_defend[1]);
  home[3]=goal_defend[3];
  return home;
end

function update()
  local t = Body.get_time();
  pose = wcm.get_pose();
  home =getHomePose();
  homeRelative = util.pose_relative(home, {pose.x, pose.y, pose.a});
  rhome = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);
  attackBearing = wcm.get_attack_bearing();
  vx,vy,va=0,0,0;

  if phase==0 then 
    if t - t0 < tstart then 
      walk.set_velocity(0,0,0);
      return;
    else walk.start();
      phase=1;
    end
  elseif phase==1 then --Approach phase 
    vx = maxStep * homeRelative[1]/rhome;
    vy = maxStep * homeRelative[2]/rhome;
    va = .2 * math.atan2(homeRelative[2], homeRelative[1]);
    if rhome < rClose then phase=2; end
  elseif phase==2 then --Turning phase, face center
    vx = maxStep * homeRelative[1]/rhome;
    vy = maxStep * homeRelative[2]/rhome;
    va = .2*attackBearing;
  end

  --Check the nearby obstacle
  obstacle_num = ocm.get_ob_num();
  obstacle_x = ocm.get_ob_x();
  obstacle_y = ocm.get_ob_y();
  obstacle_dist = ocm.get_ob_dist();

  for i=1,obstacle_num do
--print(string.format("%d XYD:%.2f %.2f %.2f",
--i,obstacle_x[i],obstacle_y[i],obstacle_dist[i]))
    if obstacle_dist[i]<0.5 then
      local r_reject = 0.5;
      local v_reject = 0.1*math.exp(-(obstacle_dist[i]/r_reject)^2);
      vx = vx - obstacle_x[i]/obstacle_dist[i]*v_reject;
      vy = vy - obstacle_y[i]/obstacle_dist[i]*v_reject;
    end
  end
  walk.set_velocity(vx, vy, va);

  if phase~=3 and rhome < rClose and 
     math.abs(attackBearing)<thClose then 
      walk.stop(); 
      phase=3;
  end
  --To prevent robot keep walking after falling down
  if phase==3 then
      walk.stop(); 
  end
end

function exit()
end

