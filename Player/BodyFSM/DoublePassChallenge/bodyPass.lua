module(..., package.seeall);

require('Body')
require('vector')
require('Motion');
require('kick');
require('HeadFSM')
require('Config')
require('wcm')
require('walk');

--initial wait 
tStartWait = Config.fsm.bodyKick.tStartWait or 0.5;
tStartWaitMax = Config.fsm.bodyKick.tStartWaitMax or 1.0;
thGyroMag = Config.fsm.bodyKick.thGyroMag or 100; 

--headFollow delay
tFollowDelay = Config.fsm.bodyKick.tFollowDelay;

t0 = 0;
tStart = 0;
timeout = 10.0;
timeout = 7.0; 
phase=0; --0 for walk wait, 1 for init.wait, 2 for kicking, 3 for headFollow

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  tStoped=t0;
  shouldWait=walk.active;
  walk.stop();
  phase=0;   
end

function update()
  t = Body.get_time();
  if (t - t0 > timeout) then
    print("bodyPass timeout")
    return "timeout";
  end
  if walk.active then
    tStoped=t;
    return;
  end

  if phase==0 then
    if not shouldWait then phase=1;
    else
       --wait until vibration ceases
      tPassed=t-tStoped;
      imuGyr = Body.get_sensor_imuGyrRPY();
      gyrMag = math.sqrt(imuGyr[1]^2+imuGyr[2]^2);
      if tPassed>tStartWaitMax or
         (tPassed>tStartWait and gyrMag<thGyroMag) then
	phase=1;
      end
    end
  elseif phase==1 then
    checkballpos();
    tStart=t;
    Motion.event("kick");
    phase = 2;
  elseif phase==2 then
    if t-tStart > tFollowDelay then
      --Wait a bit and try find the ball
      phase=3;
      HeadFSM.sm:set_state('headKickFollow');
    end
  elseif phase==3 then
    --Wait until kick is over
    if not kick.active then
--BUG here: this block get never called 
print("bodyPass done")
      walk.still=true;
      walk.set_velocity(0, 0, 0);
      walk.start();
      return "done";
    end
  end

end

function exit()
--[[
  role=gcm.get_team_role();
  team_task_state=gcm.get_team_task_state();
  team_task_state[role]=team_task_state[role]+1;
  gcm.set_team_task_state(team_task_state);
--]]
end

function checkballpos()
  role=gcm.get_team_role();
  team_task_state=gcm.get_team_task_state();
  task_state=team_task_state[role]; -- index start with 0
  ball = wcm.get_ball();

print("Ball pos:",ball.x,ball.y);
  if (ball.y > 0) then
    kick.set_kick("PassForwardLeft");
  else
    kick.set_kick("PassForwardRight");
  end
  if role==1 and task_state==1 then
    if (ball.y > 0) then
      kick.set_kick("PassForwardLeft2");
    else
      kick.set_kick("PassForwardRight2");
    end
  end
end
