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
noball_threshold = 3; 
--How many succeding ball observations is needed before updating?
--We need at least two observation to update velocity

ball_threshold = 2;

gamma = 0.3;
discount = 0.95;
oldx,oldy=0,0;
olda,oldR = 0,0;

function entry()
  oldx,oldy,vx,vy,isdodge=0,0,0,0,0;
  vxOld,vyOld = 0,0;
  tLast=Body.get_time();
  noball_count=1;
end

function update(newx,newy)
  t=Body.get_time();
  ball_count = ball_count + 1;
  ballOldR = math.sqrt(oldx^2+oldy^2);
  ballR = math.sqrt(newx^2+newy^2);
  ballA = math.atan2(newy,newx);

  gamma = 0.7;

  --Lower gamma if head not locked on at the ball
  locked_on = wcm.get_ball_locked_on();
  if locked_on==0 then
    vx,vy=0,0;
  end

  --Ball seen for some continuous frames
  if t>tLast and ball_count>=ball_threshold then
      tPassed=t-tLast;

--[[
      --Filter ball distance
      thR = math.max(0,(ballR-1.0)*0.1);
      newR = oldR;
      if math.abs(oldR - ballR)>thR then
      end
--]]

      newR = ballR;
      newA = ballA;

      nfvx = (newx-oldx)/tPassed; 
      nfvy = (newy-oldy)/tPassed; 
    
      filteredx = newR * math.cos(newA);
      filteredy = newR * math.sin(newA);

      vxCurrent = (filteredx-oldx)/tPassed;
      vyCurrent = (filteredy-oldy)/tPassed;

      vMagCurrent = math.sqrt(vxCurrent^2+vyCurrent^2);

      if vMagCurrent < 4.0 and newR < 2.0 then --Kill outlier
        --Update velocity 
        vx=(1-gamma)*discount*vx + gamma*vxCurrent;
        vy=(1-gamma)*discount*vy + gamma*vyCurrent;
        vxOld = vxCurrent;
        vyOld = vyCurrent;

        --Update last ball position
        oldx=filteredx;
        oldy=filteredy;
        oldA=newA;
        oldR=newR; 
        tLast=t;
--[[
        if math.sqrt(nfvx^2+nfvy^2)>0.3 and nfvx<-0.3 then
           print(string.format(
"%d Ball xy:%.2f %.2f vc: %.2f %.2f vcf:%.2f %.2f v:%.2f %.2f",
  	  ball_count,newx,newy,nfvx,nfvy,vxCurrent,vyCurrent,vx,vy));
        end
--]]
      else
	ball_count=0;
	--This should be a outlier
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
  else
   vx=gamma*vx;
   vy=gamma*vy;
  end
end

function getVelocity()
  return vx, vy, isdodge;
end
