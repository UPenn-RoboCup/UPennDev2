module(..., package.seeall);

require('Body')
require('walk')

t0 = 0;
timeout = Config.falling_timeout or 0.3;

function entry()
  print(_NAME.." entry");

  -- relax all the joints while falling
  Body.set_body_hardness(0);
  t0 = Body.get_time();
  Body.set_syncread_enable(1); --OP specific

  walk.stance_reset();--reset current stance
end

function update()
  local t = Body.get_time();

  -- set the robots command joint angles to thier current positions
  --  this is needed to that when the hardness is re-enabled
  local qSensor = Body.get_sensor_position();
  Body.set_actuator_command(qSensor);

  if (t-t0 > timeout) then
    return "done"
  end
end

function exit()
end
