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

started = false;
kickable = true;
follow = false;

goalie_dive = Config.goalie_dive or 0;

tStartDelay = Config.fsm.bodyAnticipate.tStartDelay;
rCloseDive = Config.fsm.bodyAnticipate.rCloseDive;
rMinDive = Config.fsm.bodyAnticipate.rMinDive;
ball_velocity_thx = Config.fsm.bodyAnticipate.ball_velocity_thx;
ball_velocity_th = Config.fsm.bodyAnticipate.ball_velocity_th;
center_dive_threshold_y = Config.fsm.bodyAnticipate.center_dive_threshold_y;
dive_threshold_y = Config.fsm.bodyAnticipate.dive_threshold_y;

ball_velocity_th2 = Config.fsm.bodyAnticipate.ball_velocity_th2;
rClose = Config.fsm.bodyAnticipate.rClose;
rCloseX = Config.fsm.bodyAnticipate.rCloseX;

timeout = Config.fsm.bodyAnticipate.timeout;
thFar = Config.fsm.bodyAnticipate.thFar or {0.4,0.4,15*math.pi/180};
maxPosition = 0.55;

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
  pose = wcm.get_pose();
  tBall = Body.get_time() - ball.t;
  ballGlobal = util.pose_global({ball.x, ball.y, 0}, {pose.x, pose.y, pose.a});
  ballR = math.sqrt(ball.x^2+ ball.y^2);

  -- See where our home position is...
  homePosition=getGoalieHomePosition();
  homeRelative = util.pose_relative(homePosition, {pose.x, pose.y, pose.a});
  rHomeRelative = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);

  goal_defend=wcm.get_goal_defend();
  ballxy=vector.new( {ball.x,ball.y,0} );
  posexya=vector.new( {pose.x, pose.y, pose.a} );
  ballGlobal=util.pose_global(ballxy,posexya);
  ballR_defend = math.sqrt(
	(ballGlobal[1]-goal_defend[1])^2+
	(ballGlobal[2]-goal_defend[2])^2);
  ballX_defend = math.abs(ballGlobal[1]-goal_defend[1]);



--TODO: Diving handling 

  ball_v = math.sqrt(ball.vx^2+ball.vy^2);

  if goalie_dive > 0 then

    if t-t0>tStartDelay and t-ball.t<0.1 then
      ballR=math.sqrt(ball.x^2+ball.y^2);
      if ball.vx<ball_velocity_thx and 
	ballR_defend<rCloseDive and
	ballR_defend>rMinDive and
        ball_v>ball_velocity_th then

        t0=t;
        py = ball.y - (ball.vy/ball.vx) * ball.x;
        print("Ball velocity:",ball.vx,ball.vy);
        print("Projected y pos:",py);
        if math.abs(py)<dive_threshold_y then
          if py>center_dive_threshold_y then 
  	    Speak.talk('Left');
            dive.set_dive("diveLeft");
          elseif py<-center_dive_threshold_y then
  	    Speak.talk('Right');
            dive.set_dive("diveRight");
          else 
	    Speak.talk('Center');
            dive.set_dive("diveCenter");
          end
          Motion.event("dive");
          return "dive";
	end
      end
    end
  end

  --TODO: check if other player is close to the ball
  if (ballR_defend<rClose or ballX_defend<rCloseX)
     and t-ball.t<0.1 and ball_v < ball_velocity_th2 then

    Motion.event("walk");
    return "ballClose";
  end

  attackBearing = wcm.get_attack_bearing();
  if Config.fsm.goalie_reposition==1 then --check yaw error only
    if (t - t0 > timeout) and 
	math.abs(attackBearing) > thFar[3] then
      Motion.event("walk");
      return 'position';
    end

  elseif Config.fsm.goalie_reposition==2 then --check yaw and position error
    if (t - t0 > timeout) and 
	( rHomeRelative>math.sqrt(thFar[1]^2+thFar[2]^2) or
	math.abs(attackBearing) > thFar[3]) then
      Motion.event("walk");
      return 'position';
    end
  end
end
function getGoalieHomePosition()

  -- define home goalie position (in front of goal and facing the ball)
  --homePosition = 1.0*vector.new(wcm.get_goal_defend());
  homePosition = 0.98*vector.new(wcm.get_goal_defend());

--[[
  vBallHome = math.exp(-math.max(tBall-3.0, 0)/4.0)*(ballGlobal - homePosition);
  rBallHome = math.sqrt(vBallHome[1]^2 + vBallHome[2]^2);

  if (rBallHome > maxPosition) then
    scale = maxPosition/rBallHome;
    vBallHome = scale*vBallHome;
  end
  homePosition = homePosition + vBallHome;
--]]


  homePosition = 0.94*vector.new(wcm.get_goal_defend());


  return homePosition;
end


function exit()
  walk.start();
end
