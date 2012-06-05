module(..., package.seeall);

require('Body')
require('walk')
require('vector')
require('util')
require('Config')
require('wcm')
require('gcm')
require('UltraSound')

t0 = 0;

--[[
maxStep = Config.fsm.bodyChase.maxStep;
tLost = Config.fsm.bodyChase.tLost;
timeout = Config.fsm.bodyChase.timeout;
rClose = Config.fsm.bodyChase.rClose;
--]]

timeout = 20.0;
maxStep = 0.04;
maxPosition = 0.55;
tLost = 6.0;

rClose = Config.fsm.bodyAnticipate.rClose;
rCloseX = Config.fsm.bodyAnticipate.rCloseX;
thClose = Config.fsm.bodyGoaliePosition.thClose;

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
end

function update()
  local t = Body.get_time();

  ball = wcm.get_ball();
  pose = wcm.get_pose();
  ballGlobal = util.pose_global({ball.x, ball.y, 0}, {pose.x, pose.y, pose.a});
  tBall = Body.get_time() - ball.t;

  homePosition=getGoalieHomePosition();

  homeRelative = util.pose_relative(homePosition, {pose.x, pose.y, pose.a});
  rHomeRelative = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);

  vx = maxStep*homeRelative[1]/rHomeRelative;
  vy = maxStep*homeRelative[2]/rHomeRelative;

  --TODO: Goalie may need to turn to target direction 
  va = .35*wcm.get_attack_bearing();

--[[
  if (tBall > 8) then
    --When ball is lost, face opponents' goal
    va = .35*wcm.get_attack_bearing();
  else
    --Face the ball
    va = math.atan2(ball.y, ball.x);
  end
--]]


  goal_defend=wcm.get_goal_defend();
  ballxy=vector.new( {ball.x,ball.y,0} );
  posexya=vector.new( {pose.x, pose.y, pose.a} );
  ballGlobal=util.pose_global(ballxy,posexya);
  ballR_defend = math.sqrt(
	(ballGlobal[1]-goal_defend[1])^2+
	(ballGlobal[2]-goal_defend[2])^2);
  ballX_defend = math.abs(ballGlobal[1]-goal_defend[1]);

  if (ballR_defend<rClose or ballX_defend<rCloseX) and tBall<1.0 then
    return "ballClose";
  end

  local tThreshClose = math.sqrt(thClose[1]^2+thClose[2]^2);
  if math.abs(homeRelative[1])<thClose[1] and
     math.abs(homeRelative[2])<thClose[2] and
     math.abs(va)<thClose[3] then
    return "ready";
  end

  walk.set_velocity(vx, vy, va);

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

  --Fixed goalie position
  homePosition = 0.94*vector.new(wcm.get_goal_defend());



  return homePosition;
end

function exit()
end

