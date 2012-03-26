module(..., package.seeall);

require('Config');
require('vector');
require('Body');

-------------------------------
-- A very simple velocity filter
-------------------------------

x,y,vx,vy,isdodge=0,0,0,0,0;

noball_count = 1;
noball_threshold = 3; 
gamma = 0.3;

function entry()
  tLast=Body.get_time();
  noball_count=1;
end

function update(newx,newy)
  --We need at least two observation to update velocity
  t=Body.get_time();
  if noball_count==0 then
    tPassed=t-tLast;
    vxCurrent = (newx-x)/tPassed;
    vyCurrent = (newy-y)/tPassed;
    vx=(1-gamma)*vx + gamma*vxCurrent;
    vy=(1-gamma)*vy + gamma*vyCurrent;
  end
  tLast=t;
  x=newx;y=newy;
  noball_count=0;
end

function update_noball()
  noball_count=noball_count+1;
  if noball_count>noball_threshold then
    vx=0;vy=0;
  end
end

function getVelocity()
  return vx, vy, isdodge;
end
