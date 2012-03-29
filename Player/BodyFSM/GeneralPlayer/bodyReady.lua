module(..., package.seeall);

require('Body')
require('walk')
require('util')
require('vector')
require('Config')
require('wcm')
require('gcm')

t0 = 0;

maxStep = Config.fsm.bodyReady.maxStep;
rClose = Config.fsm.bodyReady.thClose[1];
thClose = Config.fsm.bodyReady.thClose[2];

-- don't start moving right away
tstart = 2.0;

phase=0; --0 for wait, 1 for approach, 2 for turn, 3 for end

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  phase=0;
end

function getHomePose()
  id = gcm.get_team_player_id();
  if gcm.get_game_kickoff() == 1 then
    if (id == 1) then
      -- goalie
      home = wcm.get_goal_defend();
    elseif (id == 2) then
      -- attacker 
      home = wcm.get_goal_defend();
      home[1] = util.sign(home[1]) * .5;
    elseif (id == 3) then
      -- defender
      home = wcm.get_goal_defend();
      home[1] = util.sign(home[1]) * 1.5;
      home[2] = -0.5;
    else
      -- support
      home = wcm.get_goal_defend();
      home[1] = util.sign(home[1]) * .5;
      home[2] = 1.0;
    end
  else
    if (id == 1) then
      -- goalie
      home = wcm.get_goal_defend();
    elseif (id == 2) then
      -- attacker 
      home = wcm.get_goal_defend();
      home[1] = util.sign(home[1]) * 1.75;
    elseif (id == 3) then
      -- defender
      home = wcm.get_goal_defend();
      home[1] = util.sign(home[1]) * 1.75;
      home[2] = 1.0;
    else
      -- support
      home = wcm.get_goal_defend();
      home[1] = util.sign(home[1]) * 1.75;
      home[2] = -1.0;
    end
  end
  return home;
end

function update()
  local t = Body.get_time();
  pose = wcm.get_pose();

  if phase==0 then 
    if t - t0 < tstart then 
      return;
    else walk.start();
      phase=1;
    end
  elseif phase==1 then --Approach phase 
    home =getHomePose();
    homeRelative = util.pose_relative(home, {pose.x, pose.y, pose.a});

    rhome = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);
    vx = maxStep * homeRelative[1]/rhome;
    vy = maxStep * homeRelative[2]/rhome;
    va = .2 * math.atan2(homeRelative[2], homeRelative[1]);
    walk.set_velocity(vx, vy, va);
    if rhome < rClose then phase=2; end
  elseif phase==2 then --Turning phase, face center
    attackBearing = wcm.get_attack_bearing();
    va = .2*attackBearing;
    walk.set_velocity(vx, vy, va);
    if math.abs(attackBearing)<thClose then 
      walk.stop(); 
      phase=3;
    end
  end 

end

function exit()
end

