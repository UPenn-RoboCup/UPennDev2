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

  if( walk.stance~=0 ) then
    print('switcing stance')
    walk.switch_stance(0)
  end

end

function update()
  local t = Body.get_time();

  if( t-t0>timeout ) then
    return 'timeout'
  end

  if( boxercm.get_body_enabled() == 1 ) then
    return "done";
  end

end

function exit()
end
