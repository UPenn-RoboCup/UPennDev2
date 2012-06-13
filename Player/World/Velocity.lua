module(..., package.seeall);

require('Config');
require('vector');
require('Body');

-------------------------------
-- A very simple velocity filter
-------------------------------


noball_count = 1;
ball_count = 0;
--If ball is not seen for this # of frames, remove ball memory 
noball_threshold = 5; 
--How many succeding ball observations is needed before updating?
--We need at least two observation to update velocity
ball_threshold = 2;

gamma = 0.9;
discount = 0.95;

max_distance = 2.0; --Only check velocity within this radius
max_velocity = 4.0; --Ignore if velocity exceeds this


oldx,oldy=0,0;
olda,oldR = 0,0;
newA,newR = 0,0;

--Now we maintain a cue of ball distance
--Current ball distance is the minimum one
ballR_cue_length = 10;
ballR_cue=vector.zeros(ballR_cue_length);
ballR_index = 1;
min_ballR_old = 0;

function entry()
  oldx,oldy,vx,vy,isdodge=0,0,0,0,0;
  vxOld,vyOld = 0,0;
  tLast=Body.get_time();
  noball_count=1;
end

function update(newx,newy)
  t=Body.get_time();
  ball_count = ball_count + 1;
  ballR = math.sqrt(newx^2+newy^2);
  ballA = math.atan2(newy,newx);

  --Lower gamma if head not locked on at the ball
  locked_on = wcm.get_ball_locked_on();
  if locked_on==0 then
--    vx,vy=0,0;
  end

  --Ball 

  --Keep the history of old ball distances and pick smallest one
  min_ballR = 999;
  for i=1,ballR_cue_length do
    if ballR_cue[i]<min_ballR then
      min_ballR = ballR_cue[i];
    end
  end

  if ballR < min_ballR then --Ball approaching
    newR = ballR;
    oldR = min_ballR;
  else --Ball not approaching
    newR = min_ballR;
    oldR = min_ballR;
  end

  --Ball seen for some continuous frames
  if t>tLast and ball_count>=ball_threshold then
      tPassed=t-tLast;

      newR = ballR;
      newA = ballA;
    
      filteredx = newR * math.cos(ballA);
      filteredy = newR * math.sin(ballA);

      vxCurrent = (filteredx-oldx)/tPassed;
      vyCurrent = (filteredy-oldy)/tPassed;

      vMagCurrent = math.sqrt(vxCurrent^2+vyCurrent^2);

      if vMagCurrent < max_velocity and newR < max_distance then 
        --Update velocity 
        vx=(1-gamma)*discount*vx + gamma*vxCurrent;
        vy=(1-gamma)*discount*vy + gamma*vyCurrent;

        --Update last ball position
        oldx=filteredx;
        oldy=filteredy;
        oldA=newA;
        oldR=newR; 
        tLast=t;
      else
	--This should be a outlier
	ball_count=0;
	vx=vx*discount;
	vy=vy*discount;
      end
  else 
     --Ball first seen, don't update velocity
     vx=0;vy=0;
     --Update position
     oldx=newx;
     oldy=newy;
     oldA=ballA;
     oldR=ballR; 
     tLast=t;
     noball_count=0;
  end

end

function update_noball()
  ball_count = 0;
  noball_count=noball_count+1;
  --Reset velocity if ball was not seen 
  if noball_count==noball_threshold then
    print("Velocity resetted")
    vx=0;vy=0;
    ballR_cue=vector.zeros(ballR_cue_length);
    min_ballR_old = 0;
    oldx,oldy=0,0;
  else
   vx=gamma*vx;
   vy=gamma*vy;
  end
end

function getVelocity()
  return vx, vy, isdodge;
end
