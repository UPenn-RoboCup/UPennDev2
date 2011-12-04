module(..., package.seeall);

require('Body')
require('World')
require('Config')
require('vcm')

t0 = 0;
scanT = 10; --Config.HeadFSM.sweep.tscan;
scanPeriod = 2.5;
yawmag= Config.head.yawMax*2;

function entry()
   print(_NAME.." entry");

   t0 = Body.get_time();
   headAngles = Body.get_head_position();
   if (headAngles[1] > 0) then
      direction = -1;
   else
      direction = 1;
   end
end

function update()
  local t = Body.get_time();
  local curHeadAngles = Body.get_head_position();
  local ph = ((t-t0)%scanPeriod)/scanPeriod;
  local ph2 = ph-.5;
  -- constant velocity scan
  if (ph2>0) then ph2 = -2*ph2+0.5;
  else ph2 = 2*ph2+0.5; end
  local yaw = direction*ph2*yawmag/2;
  -- Zero on the outsides, our angle in the middle
  local headpitch = (curHeadAngles[2]-Config.head.cameraAngle[1][2]); 

  Body.set_head_command({yaw, headpitch});

  if (t - t0 > scanT) then
    return "timeout";
  end
--[[
  local wholeblock = vcm.get_freespace_block();
  if (wholeblock == 0) then
	return "done";
  end
--]]
end

function exit()
end
