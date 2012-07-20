module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
require('vector')

t0 = 0;
timeout = 5;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();

end

function update()
  local t = Body.get_time();

  -- Check if there is a punch activated
  local punch = 0;

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

end

function exit()
  print('Exiting rand box!')
end
