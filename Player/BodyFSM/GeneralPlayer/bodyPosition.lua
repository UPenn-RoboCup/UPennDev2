module(..., package.seeall);

require('Body')
require('World')
require('walk')
require('vector')
require('wcm')
require('Config')
require('Team')
require('util')

t0 = 0;
maxStep1 = Config.fsm.bodyPosition.maxStep;
maxStep2 = Config.fsm.bodyPosition.maxStep2;
tLost = Config.fsm.bodyPosition.tLost;
timeout = Config.fsm.bodyPosition.timeout;

rTurn= Config.fsm.bodyPosition.rTurn;
rTurn2= Config.fsm.bodyPosition.rTurn2;
rDist1= Config.fsm.bodyPosition.rDist1;
rDist2= Config.fsm.bodyPosition.rDist2;
rOrbit= Config.fsm.bodyPosition.rOrbit;

thClose = Config.fsm.bodyPosition.thClose;
rClose= Config.fsm.bodyPosition.rClose;

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  max_speed=0;
  count=0;
  ball=wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);
  maxStep=maxStep1;

  wcm.set_kick_dir(1);--front kick default
  wcm.set_kick_type(2);--walking kick default

end


function update()
  count=count+1;

  local t = Body.get_time();
  ball=wcm.get_ball();
  pose=wcm.get_pose();

  ballR = math.sqrt(ball.x^2 + ball.y^2);

  ballxy=vector.new( {ball.x,ball.y,0} );
  posexya=vector.new( {pose.x, pose.y, pose.a} );

  ballGlobal=util.pose_global(ballxy,posexya);
  goalGlobal=wcm.get_goal_attack();
  aBallLocal=math.atan2(ball.y,ball.x); 

  aBall=math.atan2(ballGlobal[2]-pose.y, ballGlobal[1]-pose.x);
  aGoal=math.atan2(goalGlobal[2]-ballGlobal[2],goalGlobal[1]-ballGlobal[1]);

  --In what angle should we approach the ball?
  angle1=util.mod_angle(aGoal-aBall);

  role = gcm.get_team_role();
  if (role == 2) then
    homePose = getDefenderHomePose();
  elseif (role==3) then
    homePose = getSupporterHomePose();
  else
    homePose=getAttackerHomePose();	
  end

  if role==1 then
    setAttackerVelocity();
  else
    setDefenderVelocity();
  end

  if (t - ball.t > tLost) then
    return "ballLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end

  tBall=0.5;

  if math.abs(homeRelative[1])<thClose[1] and
    math.abs(homeRelative[2])<thClose[2] and
    math.abs(homeRelative[3])<thClose[3] and
    ballR<rClose and
    t-ball.t<tBall then
      print("bodyPosition done")
      return "done";
  end
end

function setAttackerVelocity()
  uPose=vector.new({pose.x,pose.y,pose.a})
  homeRelative = util.pose_relative(homePose, uPose);  
  rHomeRelative = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);
  aHomeRelative = math.atan2(homeRelative[2],homeRelative[1]);
  homeRot=math.abs(homeRelative[3]);

  --Distance-specific velocity generation
  if rHomeRelative>0.6 and homeRot<45*math.pi/180 then
    maxStep = maxStep2;
    if max_speed==0 and homeRot<30*math.pi/180 then
      maxStep=maxStep2; --front dash
      max_speed=1;
      print("MAXIMUM SPEED")
--    Speak.play('./mp3/max_speed.mp3',50)
    end
  elseif rHomeRelative>0.4 and homeRot<45*math.pi/180 then
    if max_speed==0 and 
      homeRot<30*math.pi/180 then
      maxStep=maxStep2; --front dash
      max_speed=1;
--      Speak2.play('./mp3/max_speed.mp3',50)
    end
  else
    maxStep = maxStep1;
  end

  --Basic chase code
  vx,vy,va=0,0,0;
  aTurn=math.exp(-0.5*(rHomeRelative/rTurn)^2);

  vx = maxStep*homeRelative[1]/rHomeRelative;
  if rHomeRelative>0.8 then vy=0;
  elseif rHomeRelative>0.6 then
    vy = 0.3*maxStep*homeRelative[2]/rHomeRelative;
  else
    if math.abs(aHomeRelative)>45*180/math.pi then
      vy = maxStep*homeRelative[2]/rHomeRelative;
      aTurn=0;
    else
      vy = 0.3*maxStep*homeRelative[2]/rHomeRelative;
    end	
  end

  scale = math.min(maxStep/math.sqrt(vx^2+vy^2), 1);
  vx,vy = scale*vx,scale*vy;
  va=0.5*(aTurn*homeRelative[3] + (1-aTurn)*aHomeRelative);
  vx=math.max(0,vx) --Don't allow the robot to backstep
  walk.set_velocity(vx,vy,va);
end

function setDefenderVelocity()
  homeRelative = util.pose_relative(homePose, {pose.x, pose.y, pose.a});
  rHomeRelative = math.sqrt(homeRelative[1]^2 + homeRelative[2]^2);

  vx = maxStep*homeRelative[1]/rHomeRelative;
  vy = maxStep*homeRelative[2]/rHomeRelative;
  va = .5*math.atan2(ball.y, ball.x + 0.05);
  walk.set_velocity(vx,vy,va);
end

function getAttackerHomePose()

  if math.abs(angle1)<math.pi/2 then
    rDist=math.min(rDist1,math.max(rDist2,ballR-rTurn2));
    local homepose={
	ballGlobal[1]-math.cos(aGoal)*rDist,
	ballGlobal[2]-math.sin(aGoal)*rDist,
	aGoal};
    return homepose;
  elseif angle1>0 then
    local homepose={
	ballGlobal[1]+math.cos(-aBall+math.pi/2)*rOrbit,
	ballGlobal[2]-math.sin(-aBall+math.pi/2)*rOrbit,
	aBall};
    return homepose;

  else
    local homepose={
	ballGlobal[1]+math.cos(-aBall-math.pi/2)*rOrbit,
	ballGlobal[2]-math.sin(-aBall-math.pi/2)*rOrbit,
	aBall};
    return homepose;
  end
end

function getDefenderHomePose()
    -- defend
  homePosition = .6 * ballGlobal;
  homePosition[1] = homePosition[1] - 0.50*util.sign(homePosition[1]);
  homePosition[2] = homePosition[2] - 0.80*util.sign(homePosition[2]);
  return homePosition;
end

function getSupporterHomePose()

    -- support
    attackGoalPosition = vector.new(wcm.get_goal_attack());

    --[[
    homePosition = ballGlobal;
    homePosition[1] = homePosition[1] + 0.75*util.sign(homePosition[1]);
    homePosition[1] = util.sign(homePosition[1])*math.min(2.0, math.abs(homePosition[1]));
    homePosition[2] = 
    --]]

    -- move near attacking goal
    homePosition = attackGoalPosition;
    -- stay in the field (.75 m from end line)
    homePosition[1] = homePosition[1] - util.sign(homePosition[1]) * 1.0;
    -- go to far post (.75 m from center)
    homePosition[2] = -1*util.sign(ballGlobal[2]) * .75;

    -- face ball 
    homePosition[3] = ballGlobal[3];
    return homePosition;
end

function exit()
end

function sign(s)
  if s>0 then return 1;
  else return -1;
  end
end

