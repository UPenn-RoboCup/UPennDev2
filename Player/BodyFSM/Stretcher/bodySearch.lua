module(..., package.seeall);

require('Body')
require('wcm')
require('walk')
ps = false;

if( Config.stretcher.primesense and Config.game.playerID==1 ) then
  print('Using the PrimeSense for control!')  
  require 'primecm'    
  ps = true;
end

t0 = 0;
timeout = 10.0;

-- turn velocity
vSpin = 0.3;
direction = 1;
-- maximum walk velocity
maxStep = 0.04;

function entry()
  print("Body FSM:".._NAME.." entry");

  t0 = Body.get_time();

  if( not ps ) then  
    -- set turn direction to last known ball position
    stretcher = wcm.get_stretcher();
    if (stretcher.y > 0) then
      direction = 1;
    else
      direction = -1;
    end
  end
end

function update()
  local t = Body.get_time();

  if( ps ) then
    if( primecm.get_skeleton_found()==1 ) then
      print('Updating via PrimeSense')
      local torso = primecm.get_skeleton_torso();
      local vx = -1*torso[3] - 240;
      local vy = -1*torso[1] - 200;
      local va = 0;
      scale = math.min(maxStep/math.sqrt(vx^2+vy^2), 1);
      vx = vx * scale;
      vy = vy * scale;
      print('torso',unpack(torso))
      print('vel:',vx,vy)
      print()
      walk.set_velocity( vx, vy, va );
    else
      print('User not found...')
      walk.set_velocity( 0,0,0 );      
    end
  else
    stretcher = wcm.get_stretcher();    
    -- search/spin until the ball is found
    walk.set_velocity(0, 0, direction*vSpin);

    if (t - stretcher.t < 0.1) then
      return "stretcher";
    end
    if (t - t0 > timeout) then
      return "timeout";
    end

  end

end

function exit()
end
