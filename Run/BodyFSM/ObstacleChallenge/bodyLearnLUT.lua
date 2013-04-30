module(..., package.seeall);

local Body = require('Body')
local walk = require('walk')
local vector = require('vector')

local wcm = require('wcm')
local gcm = require('gcm')

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

