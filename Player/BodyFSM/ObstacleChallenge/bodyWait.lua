module(..., package.seeall);

require('Body')
require('Motion')

t0 = 0
timeout = 60.0;

function entry()
  print(_NAME..' entry');

  walk.set_velocity(0,0,0);
  walk.stop();
end

function update()
  local t = Body.get_time();

  if (t - t0 > timeout) then
    return 'timeout'
  end

  local flag = vcm.get_camera_learned_new_lut();
  if flag == 1 then
    print('Learned LUT, Start Body FSM')
    return 'done'
  end

end

function exit()
  Motion.sm:add_event('walk');
end
