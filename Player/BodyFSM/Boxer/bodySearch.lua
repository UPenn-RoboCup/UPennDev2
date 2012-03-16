module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')

t0 = 0;
timeout = 2.5;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();

  -- Set move
  move = math.floor(math.random()*3);
  moved = false;
--  walk.switch_stance( 1 );
  print('Move',move)
end

function update()
  local t = Body.get_time();


  -- search/spin until the ball is found
--  walk.set_velocity(0, 0, direction*vSpin);
if( not moved ) then
  moved = true;
	if( move==0 ) then --switch stance
		stance_dir = math.floor(math.random()*2)+1;
if(math.floor(math.random()*5)==0) then
walk.switch_stance( 0 );
else
                walk.switch_stance( stance_dir );
end
	else
		punch_type = math.floor(math.random()*2)+1;
                walk.doPunch(punch_type);
	end
else
  if (t - t0 > timeout) then
    return "timeout";
  end
end

end

function exit()
  print('Exiting rand box!')
end
