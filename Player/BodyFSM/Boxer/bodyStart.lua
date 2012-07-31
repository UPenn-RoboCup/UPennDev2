module(..., package.seeall);

require('Body')
require('boxercm')
require('walk')
require('vector')

t0 = 0;
timeout = 5;

boxMode = 'mimic'

function entry()
  print("Body FSM:".._NAME.." entry");
  t0 = Body.get_time();

  walk.set_velocity(0,0,0);
  walk.stop();
  started = false;

end

function update()
  local t = Body.get_time();

  if not started then
    if not walk.active then
      Motion.sm:set_state('standstill');
      started = true;
    end
  end

  if( t-t0>timeout ) then
    return 'timeout'
  end

  if( boxercm.get_body_enabled() == 1 ) then
    if(boxMode=='mimic') then
      return "doneMimic";
    else
      return "doneBox"
    end
  end

end

function exit()
--  Motion.sm:add_event('walk');
end
