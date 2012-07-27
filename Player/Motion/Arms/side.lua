module(..., package.seeall);

myplayer = 0;
myhand = 'right'
function init(forPlayer)
  myplayer = forPlayer;
  primecm = require('primecm'..forPlayer)
  boxercm = require('boxercm'..forPlayer)
end

function entry()
  print("Boxer ".._NAME.." entry");
  t0 = unix.time();
  if(myhand=='left') then
    boxercm.set_body_punchL(0);
  else
    boxercm.set_body_punchR(0);
  end
end

function update()

  t = unix.time();

  -- TODO: Need to check the confidence values!
  if(myhand=='left') then
    e2h = primecm.get_position_ElbowL() - primecm.get_position_HandL();
    s2e = primecm.get_position_ShoulderL() - primecm.get_position_ElbowL();
    s2h = primecm.get_position_ShoulderL() - primecm.get_position_HandL();
  else
    e2h = primecm.get_position_ElbowR() - primecm.get_position_HandR();
    s2e = primecm.get_position_ShoulderR() - primecm.get_position_ElbowR();
    s2h = primecm.get_position_ShoulderR() - primecm.get_position_HandR();
  end
  -- Change to OP coordinates
  --[[
  local left_hand  = vector.new({s2hL[3],s2hL[1],s2hL[2]}) / arm_lenL; -- z is OP x, x is OP y, y is OP z
  --]]
  arm_len = vector.norm( e2h ) + vector.norm( s2e );
  hand = vector.new({s2h[3],s2h[1],-1*s2h[2]}) / arm_len;

  -- Check if the hand extends beyond a certain point
  if( hand[3]>.4 ) then
    return 'up'
  elseif(hand[1]>.9) then
    return 'forward';
  end

end

function exit()
  print("Boxer ".._NAME.." exit");  
end
