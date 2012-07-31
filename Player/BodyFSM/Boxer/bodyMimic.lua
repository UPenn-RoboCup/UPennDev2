module(..., package.seeall);

require('Body')
require('boxercm')
require('walk')
require('vector')

t0 = 0;
timeout = 5;

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();
Motion.sm:add_event('walk');
end

function update()
  local t = Body.get_time();

  -- Check if there is a punch activated
  local qL = boxercm.get_body_qLArm();
  local qR = boxercm.get_body_qRArm();
  local rpy = boxercm.get_body_rpy();

  -- Add the override
  walk.upper_body_override(qL, qR, rpy);

  if( boxercm.get_body_enabled() == 0 ) then
    print('Boxing disabled!')
    return "disabled";
  end

end

function exit()
  walk.upper_body_override_off()
end
