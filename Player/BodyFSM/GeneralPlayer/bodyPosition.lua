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

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  max_speed=0;
  count=0;
  ball=wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);
  maxStep=maxStep1;
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

  homePose=getAttackerHomePose();	

  uPose=vector.new({pose.x,pose.y,pose.a})
  pRelative=util.pose_relative(homePose,uPose);
  rRelative=math.sqrt(pRelative[1]^2+pRelative[2]^2);
  aRelative=math.atan2(pRelative[2],pRelative[1]);

  --Basic chase code
  vStep = vector.new({0,0,0});
  aTurn=math.exp(-0.5*(rRelative/rTurn)^2);

  --TODO: generalize   
  if rRelative>0.6 and 
    math.abs(pRelative[3])<45*math.pi/180 
  then
      maxStep = maxStep2;
      if max_speed==0 and 
        math.abs(pRelative[3])<30*math.pi/180 
      then
        max_speed=1;
	print("MAXIMUM SPEED")
--        Speak.play('./mp3/max_speed.mp3',50)
      end
  elseif rRelative>0.4 and 
    math.abs(pRelative[3])<45*math.pi/180  
  then
    if max_speed==0 and 
      math.abs(pRelative[3])<30*math.pi/180 then
      maxStep=maxStep2; --front dash
      max_speed=1;
--      Speak2.play('./mp3/max_speed.mp3',50)
    end
  else
    maxStep = maxStep1;
  end

  vStep[1] = maxStep*pRelative[1]/rRelative;
  if rRelative>0.8 then
    vStep[2]=0;
  elseif rRelative>0.6 then
    vStep[2] = 0.3*maxStep*pRelative[2]/rRelative;
  else
    if math.abs(aRelative)>45*180/math.pi then
      vStep[2] = maxStep*pRelative[2]/rRelative;
      aTurn=0;
    else
      vStep[2] = 0.3*maxStep*pRelative[2]/rRelative;
    end	
  end

  scale = math.min(maxStep/math.sqrt(vStep[1]^2+vStep[2]^2), 1);
  vStep = scale*vStep;
  vStep[3]=0.5*(aTurn*pRelative[3] + (1-aTurn)*aRelative);
  vStep[1]=math.max(0,vStep[1]) --Don't allow the robot to backstep
  walk.set_velocity(vStep[1],vStep[2],vStep[3]);

  if (t - ball.t > tLost) then
    return "ballLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end

  tBall=0.5;

  if math.abs(pRelative[1])<thClose[1] and
    math.abs(pRelative[2])<thClose[2] and
    math.abs(pRelative[3])<thClose[3] and
    t-ball.t<tBall then
      return "done";
  end
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

function exit()
end

function sign(s)
  if s>0 then return 1;
  else return -1;
  end
end

