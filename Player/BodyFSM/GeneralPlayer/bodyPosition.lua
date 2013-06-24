module(..., package.seeall);

require('Body')
require('World')
require('walk')
require('vector')
require('wcm')
require('Config')
require('Team')
require('util')
require('walk')

require('behavior')
require('position')

require('UltraSound');

t0 = 0;


tLost = Config.fsm.bodyPosition.tLost;
timeout = Config.fsm.bodyPosition.timeout;
thClose = Config.fsm.bodyPosition.thClose;
rClose= Config.fsm.bodyPosition.rClose;
fast_approach=Config.fsm.fast_approach or 0;
test_teamplay = Config.team.test_teamplay or 0;


avoid_ultrasound = Config.team.avoid_ultrasound or 0;

last_ph = 0;


ROLE_GOALIE = 0;
ROLE_ATTACKER = 1;
ROLE_DEFENDER = 2;
ROLE_SUPPORTER = 3;
ROLE_DEFENDER2 = 4;
ROLE_CONFUSED = 5;



function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
  max_speed=0;
  count=0;
  ball=wcm.get_ball();
  ballR = math.sqrt(ball.x^2 + ball.y^2);
  maxStep=maxStep1;
  behavior.update();

  step_count = 0;
end


function update()

  count=count+1;

  local t = Body.get_time();
  ball=wcm.get_ball();
  pose=wcm.get_pose();
  ballR = math.sqrt(ball.x^2 + ball.y^2);

  --recalculate approach path when ball is far away
  if ballR>0.60 then
    behavior.update();
  end

  --Current cordinate origin: midpoint of uLeft and uRight
  --Calculate ball position from future origin
  --Assuming we stop at next step
  if fast_approach ==1 then
    uLeft = walk.uLeft;
    uRight = walk.uRight;
    uFoot = util.se2_interpolate(0.5,uLeft,uRight); --Current origin 
    if walk.supportLeg ==0 then --left support 
      uRight2 = walk.uRight2;
      uLeft2 = util.pose_global({0,2*walk.footY,0},uRight2);
    else --Right support
      uLeft2 = walk.uLeft2;
      uRight2 = util.pose_global({0,-2*walk.footY,0},uLeft2);
    end
    uFoot2 = util.se2_interpolate(0.5,uLeft2,uRight2); --Projected origin 
    uMovement = util.pose_relative(uFoot2,uFoot);
    uBall2 = util.pose_relative({ball.x,ball.y,0},uMovement);
    ball.x=uBall2[1];
    ball.y=uBall2[2];
  else
  end

  role = gcm.get_team_role();
  kickDir = wcm.get_kick_dir();

  if role==ROLE_GOALIE then return "goalie";  end

  is_confused = wcm.get_robot_is_confused();

  --Force attacker for demo code
  if Config.fsm.playMode==1 then role = ROLE_ATTACKER; 
    homePose = position.getDirectAttackerHomePose();
  elseif (role == ROLE_DEFENDER) then
    homePose = position.getDefenderHomePose();
  elseif (role == ROLE_DEFENDER2) then
    homePose = position.getDefenderLeftHomePose();
  elseif (role == ROLE_SUPPORTER) then
    homePose = position.getSupporterHomePose();
  elseif (role == ROLE_CONFUSED) then
    homePose = position.getConfusedHomePose();
  else --Attacker
    --We don't care to turn when we do sidekick
    if Config.fsm.playMode~=3 or kickDir~=1 or is_confused>0 then 
      homePose = position.getDirectAttackerHomePose();
    else
      homePose = position.getAttackerHomePose();
    end	
  end

  --Field player cannot enter our penalty box

--SJ:  We replace this with potential field around goalie

--[[
  if role~=0 then
    goalDefend = wcm.get_goal_defend();
    homePose[1]=util.sign(goalDefend[1])*
	math.min(2.2,homePose[1]*util.sign(goalDefend[1]));
  end
--]]

  if role==ROLE_ATTACKER then
    vx,vy,va=position.setAttackerVelocity(homePose);
    --In teamplay test mode, immobilize attacker
    if test_teamplay==1 then
      vx,vy,va = 0,0,0;
    end
  else
    vx,vy,va=position.setDefenderVelocity(homePose);
  end

  --Get pushed away if other robots are around
  obstacle_num = wcm.get_obstacle_num();
  obstacle_x = wcm.get_obstacle_x();
  obstacle_y = wcm.get_obstacle_y();
  obstacle_dist = wcm.get_obstacle_dist();
  obstacle_role = wcm.get_obstacle_role();

  avoid_own_team = Config.team.avoid_own_team or 0;

  if avoid_own_team then
   for i=1,obstacle_num do

    --Role specific rejection radius
    if role==0 then --Goalie has the highest priority 
      r_reject = 0.4;

    elseif role==1 then --Attacker
      if obstacle_role[i]==0 then --Our goalie
--        r_reject = 1.0;
        r_reject = 0.5;


      elseif obstacle_role[i]<4 then --Our team
        r_reject = 0.001;
      else
        r_reject = 0.001;
      end
    else --Defender and supporter
      if obstacle_role[i]<4 then --Our team
        if obstacle_role[i]==0 then --Our goalie
--          r_reject = 1.0;
          r_reject = 0.7;
        else
          r_reject = 0.6;
        end
      else --Opponent team
        r_reject = 0.6;
      end
    end

    if obstacle_dist[i]<r_reject then
      local v_reject = 0.2*math.exp(-(obstacle_dist[i]/r_reject)^2);
      vx = vx - obstacle_x[i]/obstacle_dist[i]*v_reject;
      vy = vy - obstacle_y[i]/obstacle_dist[i]*v_reject;
    end
   end
  end

  if avoid_ultrasound then
    leftblocked,rightblocked = UltraSound.check_obstacle();

    if leftblocked and rightblocked then
      vx = -0.02;
      vy = 0 ;
      va = 0;
    elseif leftblocked and not rightblocked then
      vx = 0;
      vy = 0 ;
      va = -0.3;
    elseif not leftblocked and rightblocked then
      vx = 0;
      vy = 0;
      va = 0.3;
    end
  end


  if walk.ph<last_ph then 
--[[
    print(string.format("BodyPosition step %d", step_count));
    print(string.format("Ball: (%.3f, %.3f) %.2fs ago",	
			ball.x,ball.y, t-ball.t));
    print(string.format("Walk velocity:%.2f %.2f %.2f\n",vx,vy,va));
--]]
    step_count = step_count + 1;
   end
  last_ph = walk.ph;
 
  walk.set_velocity(vx,vy,va);

  if (t - ball.t > tLost) then
    return "ballLost";
  end
  if (t - t0 > timeout) then
    return "timeout";
  end

  tBall=0.5;

  attackAngle = wcm.get_goal_attack_angle2();
  daPost = wcm.get_goal_daPost2();
  daPostMargin = 15 * math.pi/180;
  daPost1 = math.max(thClose[3],daPost/2 - daPostMargin);

  uPose=vector.new({pose.x,pose.y,pose.a})
  homeRelative = util.pose_relative(homePose, uPose);  
  angleToTurn = math.max(0, homeRelative[3] - daPost1);


  --Direct kick check
  if Config.fsm.playMode~=3 or is_confused>0 then
    if math.abs(homeRelative[1])<thClose[1] and
       math.abs(homeRelative[2])<thClose[2] and
       ballR<rClose and
       t-ball.t<tBall and
			 role~=ROLE_CONFUSED  then

      --if current role is confused, we are not allowed to kick the ball
      print("bodyPosition ballClose")
      return "ballClose";
    end
  end

  --Curved approach check
  if math.abs(homeRelative[1])<thClose[1] and
    math.abs(homeRelative[2])<thClose[2] and
    math.abs(homeRelative[3])<daPost1 and
    ballR<rClose and
    t-ball.t<tBall and
	 role~=ROLE_CONFUSED  then

      print("bodyPosition done")
      return "done";
  end
end

function exit()
end

