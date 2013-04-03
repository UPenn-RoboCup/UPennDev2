module(..., package.seeall);

require('Body')
require('walk')
require('util')
require('vector')
require('Config')
require('wcm')
require('gcm')
require('Team')

t0 = 0;

maxStep = Config.fsm.bodyReady.maxStep;
rClose = Config.fsm.bodyReady.thClose[1];
thClose = Config.fsm.bodyReady.thClose[2];
tstart = 1.0;

phase=0; --0 for wait, 1 for approach, 2 for turn, 3 for end

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  phase=0;
  HeadFSM.sm:set_state('headReady');

end

function update()
  local t = Body.get_time();
  pose = wcm.get_pose();
  home = gcm.get_team_target();

  homeRelative = util.pose_relative(home, {pose.x, pose.y, pose.a});
  rhome = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);
  attackBearing = util.mod_angle(home[3]-pose.a);

  if phase==0 then 
    if t - t0 < tstart then 
      return;
    else walk.start();
      phase=1;
    end
  elseif phase==1 then --Approach phase 
    vx = maxStep * homeRelative[1]/rhome;
    vy = maxStep * homeRelative[2]/rhome;
    va = .2 * math.atan2(homeRelative[2], homeRelative[1]);
    walk.set_velocity(vx, vy, va);
    if rhome < rClose then phase=2; end
  elseif phase==2 then --Turning phase, face center
    vx = maxStep * homeRelative[1]/rhome;
    vy = maxStep * homeRelative[2]/rhome;
    va = .2*attackBearing;
    walk.set_velocity(vx, vy, va);
  end

  if phase~=3 and rhome < rClose and 
     math.abs(attackBearing)<thClose then 
      walk.stop(); 
      phase=3;
      return "done";
  end
end

function exit()
  print("In position")
  HeadFSM.sm:set_state('headTrack');
end

