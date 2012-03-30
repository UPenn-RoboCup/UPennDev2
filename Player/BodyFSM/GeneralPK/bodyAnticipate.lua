module(..., package.seeall);

require('Body')
require('vector')
require('Motion');
require('kick');
require('HeadFSM')
require('Config')
require('wcm')

require('walk');
require('dive')

t0 = 0;
tStart = 0;
timeout = 60.0;

started = false;
kickable = true;
follow = false;

tFollowDelay = Config.fsm.bodyKick.tFollowDelay;
tStartDelay = 5.0;
ball_velocity_th = -0.5;

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  started = false;
  follow = false;
  Motion.event("diveready");
end

function update()
  local t = Body.get_time();
  ball = wcm.get_ball();
  if t-t0>tStartDelay and t-ball.t<0.1 then
    --Tracking the ball in ready position. Stop off head movement
    Body.set_head_hardness(0);
    if ball.vx<ball_velocity_th then
      print("Ball velocity:",ball.vx,ball.vy);
      if ball.vy>0 then 
        dive.set_dive("diveLeft");
      else
        dive.set_dive("diveRight");
      end
      Motion.event("dive");
      return "done";
    end
  end
end

function exit()
end
