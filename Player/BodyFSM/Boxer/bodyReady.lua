module(..., package.seeall);

require('Body')
require('walk')
require('util')
require('vector')
require('Config')
require('wcm')
require('gcm')

t0 = 0;
prepmotion = 0;
do_motion = true;
phase = 0;

function entry()
  print(_NAME.." entry");
  Motion.event('standup');
end


function update()

  t = Body.get_time();

  t = Body.get_time();
  if not do_motion then
    return "done"
  end

  if walk.active then
    walk.stop();
    return;
  else

    if phase==0 then 
      print('Do motion!')
      phase = 1;
      t0 = Body.get_time();
      if prepmotion==0 then --goalie
        walk.startMotion("hurray2");
      elseif prepmotion==1 then --attacker
        walk.startMotion("point");
      elseif prepmotion==2 then --attacker
        walk.startMotion("hurray1");
      end
    elseif t-t0>3.0 then
      return "done";
    end
  end
end

function exit()
end

