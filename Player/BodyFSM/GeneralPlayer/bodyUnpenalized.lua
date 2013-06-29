module(..., package.seeall);

require('Body')
require('walk')
require('gcm')
require('wcm')
require('Speak')

t0=0;
timeout = 6.0; --2 sec standup + 4 sec walk forward

function entry()
  print(_NAME..' entry');
  t0 = Body.get_time();
end

function update()
  t=Body.get_time();
  walk.set_velocity(0.04,0,0); --Walk forward
  ball = wcm.get_ball();

  if t - ball.t<0.2 or t-t0 > timeout then
    return 'done';
  end
end


function exit()
  walk.start();
end
