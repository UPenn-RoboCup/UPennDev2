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
--Update velocity only after seeing ball for this # of frames
ball_threshold = 3; 
gamma = 0.3;

function entry()
  x,y,vx,vy,isdodge=0,0,0,0,0;
  tLast=Body.get_time();
  noball_count=1;
end

function update(newx,newy)
  --We need at least two observation to update velocity
  t=Body.get_time();
  ball_count = ball_count + 1;

  if noball_count==0 and ball_count>ball_threshold and t>tLast then
    tPassed=t-tLast;
    vxCurrent = (newx-x)/tPassed;
    vyCurrent = (newy-y)/tPassed;
    vx=(1-gamma)*vx + gamma*vxCurrent;
    vy=(1-gamma)*vy + gamma*vyCurrent;
  end

  if ball_count<15 and vx<-0.5 then
    print(string.format("%d Ball xy:%.2f %.2f v:%.2f %.2f",
	ball_count,newx,newy,vx,vy));
  end

  tLast=t;
  x=newx;y=newy;
  noball_count=0;
end

function update_noball()
  ball_count = 0;
  noball_count=noball_count+1;
  if noball_count>noball_threshold then
    vx=0;vy=0;
  end
end

function getVelocity()
  return vx, vy, isdodge;
end
