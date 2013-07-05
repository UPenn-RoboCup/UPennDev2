module(..., package.seeall);

local Body = require('Body')
local vector = require('vector')
local Motion = require('Motion');
local kick = require('kick');
local HeadFSM = require('HeadFSM')
local Config = require('Config')
local wcm = require('wcm')
local walk = require('walk');

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
  if (t - t0 > 2.0) then
    ball=wcm.get_ball();
    ballR = math.sqrt(ball.x^2+ball.y^2);
    if ballR> 0.3 then
      role=gcm.get_team_role();
      team_task_state=gcm.get_team_task_state();
      team_task_state[role]=team_task_state[role]+1;
      gcm.set_team_task_state(team_task_state);
      return "done";
    else
      return "again";
    end
  end
end

function exit()
end
