module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')
require 'libfootball'

t0 = 0;
timeout = 20.0;
-- maximum walk velocity
maxStep = 0.04;
-- opponent distance threshold
rClose = 0.35;

-- opponent detection timeout
tLost = 3.0;

function entry()
  print("Body FSM:".._NAME.." entry");

--  Speak.talk('chasing time!')

  t0 = Body.get_time();
end

function update()
  local t = Body.get_time();

  -- get opponent position
  local opose = footballcm.get_opponent_pose();
  local gps_pose = wcm.get_robot_gpspose();
  local vel = libfootball.update()
  walk.set_velocity(vel.vx, vel.vy, vel.va);

  local rOppRelative = libfootball.get_dist();
  if (rOppRelative < rClose) then
    libfootball.record_yardage();
    return "close";
  end

  if (t - t0 > timeout) then
    return "timeout";
  end

end

function exit()
end

