module(..., package.seeall);

require('Body')
require('boxercm')
require('walk')
require('vector')

t0 = 0;
timeout = 5;

roll0 = 0;
pitch0 = 0; -- Default pitch offset
yaw0 = 0;

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
end

function update()
  local t = Body.get_time();

  -- Check if there is a punch activated
  local qL = boxercm.get_body_qLArm();
  local qR = boxercm.get_body_qRArm();
  local rpy = boxercm.get_body_rpy();

  -- Add the override
  walk.upper_body_override(qL, qR, {0,walk.bodyTilt,0});

  -- Add the velocity
  local vx = rpy[2] - pitch0;
  local vy = rpy[1] - roll0;
  local va = rpy[3] - yaw0;
  walk.set_velocity({vx,vy,va});

  if( boxercm.get_body_enabled() == 0 ) then
    print('Boxing disabled!')
    return "disabled";
  end

end

function exit()
  walk.upper_body_override_off()
  walk.set_velocity({0,0,0});
end
