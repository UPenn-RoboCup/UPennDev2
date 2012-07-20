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

end

function update()
  local t = Body.get_time();

  -- Check if there is a punch activated
  local pL = boxercm.get_body_punchL();
  local pR = boxercm.get_body_punchR();
  local vel = boxercm.get_body_velocity();

  if( pL==1 and pR==0 ) then -- left arm punch
    --Check the stance
    if( walk.stance==1 ) then --left
    end
  elseif( pL==0 and pR==1 ) then -- right arm punch
  elseif( pL==1 and pR==1 ) then -- both arm punch (pushaway)
  else -- No punch
  end

  if( primecm.get_body_enabled() == 0 ) then
    return "lost";
  end

end

function exit()
end
