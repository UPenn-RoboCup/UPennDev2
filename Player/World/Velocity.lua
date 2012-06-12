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

  gamma = 0.7;

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

  print(newx,newy);

  --Ball seen for some continuous frames
  if t>tLast and ball_count>=ball_threshold then
      tPassed=t-tLast;
<<<<<<< HEAD

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
=======
>>>>>>> c2eb4bce9dad966cd0c9007fd320681b1452b047
    
      filteredx = newR * math.cos(ballA);
      filteredy = newR * math.sin(ballA);

      vxCurrent = (filteredx-oldx)/tPassed;
      vyCurrent = (filteredy-oldy)/tPassed;

<<<<<<< HEAD
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
=======
      if ballR < 3.0 then
        --Update velocity 
        vx=(1-gamma)*discount*vx + gamma*vxCurrent;
        vy=(1-gamma)*discount*vy + gamma*vyCurrent;
      end

      --Update position
      oldx=filteredx;
      oldy=filteredy;
      oldA=newA;
      oldR=newR; 

--[[
      if math.sqrt(vxCurrent^2+vyCurrent^2)>0.6 and vxCurrent<-0.6 then
         print(string.format(
"%d Ball xy:%.2f %.2f vcf:%.2f %.2f v:%.2f %.2f",
	ball_count,newx,newy,vxCurrent,vyCurrent,vx,vy));
      end
--]]

--[[
      if math.sqrt(vx^2+vy^2)>1.0 and vx<-1.0 then
         print(string.format(
"%d Ball :old %.1f %.1f new %.1f %.1f f: %.1f %.1f vcf:%.1f %.1f v:%.1f %.1f",
	
ball_count,oldx,oldy,newx,newy,filteredx,filteredy,vxCurrent,vyCurrent,vx,vy));
      end
--]]

--[[

  if math.abs(vxCurrent)>1.0 then
         print(string.format(
"%d Ball :old %.1f %.1f new %.1f %.1f f: %.1f %.1f vcf:%.1f %.1f",
ball_count,oldx,oldy,newx,newy,filteredx,filteredy,vxCurrent,vyCurrent));

"%d %f %f",
ball_count,newx,newy));

  end
--]]


>>>>>>> c2eb4bce9dad966cd0c9007fd320681b1452b047
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
<<<<<<< HEAD
=======


  --Update ball distance history
  ballR_cue[ballR_index]=ballR;
  ballR_index=ballR_index+1;
  if ballR_index>ballR_cue_length then
    ballR_index=1;
  end
  min_ballR_old = min_ballR;

  tLast=t;
  noball_count=0;
>>>>>>> c2eb4bce9dad966cd0c9007fd320681b1452b047
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
