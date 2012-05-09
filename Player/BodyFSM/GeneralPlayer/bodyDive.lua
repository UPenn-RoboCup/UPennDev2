module(..., package.seeall);

require('Body')
require('vector')
require('Motion');
require('Config')

-- This is a dummy state that just recovers from a dive
-- and catches the case when it never ends up falling...

t0 = 0;
timeout = 6.0;

function entry()
  print(_NAME.." entry");
  t0 = Body.get_time();
end

function update()
  t = Body.get_time();

  if (t - t0 > timeout) then
    print("bodyDive timeout")
    return "timeout";
  end

end

function exit()
end
