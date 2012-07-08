module(..., package.seeall);

require('Body')
require('vcm')
require('mcm')

t0 = 0;
timeout = 6.0;

function entry()
  pitchBias =  mcm.get_headPitchBias();--robot specific head bias

  print(_NAME.." entry");

  t0 = Body.get_time();

  -- set head to default position
  local yaw = 0;
  local pitch = 45*math.pi/180;

  Body.set_head_command({yaw, pitch-pitchBias});

end

function update()
  local t = Body.get_time()
  
  if (t - t0) > timeout then
    return 'timeout'
  end
end

function exit()
end

