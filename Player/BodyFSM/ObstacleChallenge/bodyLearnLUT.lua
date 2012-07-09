module(..., package.seeall);

require('Body')
require('walk')
require('vector')

require('wcm')
require('gcm')

t0 = 0;
timeout = 6.0;

function entry()
  print(_NAME.." entry");

  t0 = Body.get_time();
  walk.set_velocity(0,0,0);
  walk.stop();
end

function update()
  local t = Body.get_time();
  walk.stop();

  if (t - t0 > timeout) then
    return "timeout";
  end

  -- check if new lut learned
  local flag = vcm.get_camera_learned_new_lut();
  if (flag == 1) then
    return 'done';
  end

end

function exit()
  walk.start();
end

