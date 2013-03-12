-- CHARLI laser testing
print('Testing ARMS')

cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
require('init')

require 'carray'

require('Config')
require('Body')
require('Speak')
require('Motion')
require('vector')

-- Laser getting
--require 'WebotsLaser'
--print( "LIDAR Dim:", WebotsLaser.get_width(), WebotsLaser.get_height())
--nlidar_readings = WebotsLaser.get_width() * WebotsLaser.get_height();


require 'rcm'
--
require 'mcm'

-- Arms
require 'pickercm'
require ('Kinematics')
require 'Transform'

require 'Team' --To receive the GPS coordinates from objects
require 'wcm'


Body.set_l_gripper_command({0,0});
Body.set_r_gripper_command({0,0});
Body.set_l_gripper_hardness({1,1});
Body.set_r_gripper_hardness({1,1});

-- Initialize Variables
webots = false;
teamID   = Config.game.teamNumber;
playerID = Config.game.playerID;
print '=====================';
print('Team '..teamID,'Player '..playerID)
print '=====================';
targetvel=vector.zeros(3);
if (string.find(Config.platform.name,'Webots')) then
  print('On webots!')
  webots = true;
end
Team.entry();


-- Key Input
if( webots ) then
  controller.wb_robot_keyboard_enable( 100 );
else
  require 'getch'
  getch.enableblock(1);
end


function check_ik(tr, is_left)
  local qInv, dist, torso_arm_ik;
  if is_left>0 then
    qInv = Kinematics.inverse_l_arm(tr);
    torso_arm_ik = Kinematics.l_arm_torso(qInv);
  else
    qInv = Kinematics.inverse_r_arm(tr);
    torso_arm_ik = Kinematics.r_arm_torso(qInv);
  end
  dist = math.sqrt(
	(torso_arm_ik[1]-tr[1])^2+
	(torso_arm_ik[2]-tr[2])^2+
	(torso_arm_ik[3]-tr[3])^2);
  return qInv, dist;
end

function check_kinematics()
  print("Left arm angles:",vector.new(qLArm)*180/math.pi);
  torso_arm_ik = Kinematics.l_arm_torso(qLArm);
  print("Left arm transform xyz:",
	torso_arm_ik[1],torso_arm_ik[2],torso_arm_ik[3]);
  print("Left arm transform rpy:",
	torso_arm_ik[4]*180/math.pi,
	torso_arm_ik[5]*180/math.pi,
	torso_arm_ik[6]*180/math.pi);
  

  qInv = Kinematics.inverse_l_arm(torso_arm_ik);

  print("Inverse arm angles:",vector.new(qInv)*180/math.pi);

end





-- Process Key Inputs
function process_keyinput()
  if( webots ) then
    str = controller.wb_robot_keyboard_get_key()
    byte = str;
    -- Webots only return captal letter number
    if byte>=65 and byte<=90 then
      byte = byte + 32;
    end
  else
    str  = getch.get();
    byte = string.byte(str,1);
  end
  --print('byte: ', byte)
  --print('string: ',string.char(byte))

  if byte==0 then return false; end
	
  local update_walk_vel = false;
  local update_arm = false;

  if byte==string.byte("q") then  
    qLArm[1]=qLArm[1] + 5*math.pi/180;    
  elseif byte==string.byte("w") then  
    qLArm[2]=qLArm[2] + 5*math.pi/180;    
  elseif byte==string.byte("e") then  
    qLArm[3]=qLArm[3] + 5*math.pi/180;    
  elseif byte==string.byte("r") then  
    qLArm[4]=qLArm[4] + 5*math.pi/180;    
  elseif byte==string.byte("t") then  
    qLArm[5]=qLArm[5] + 5*math.pi/180;    
  elseif byte==string.byte("y") then  
    qLArm[6]=qLArm[6] + 5*math.pi/180;    

  elseif byte==string.byte("a") then  
    qLArm[1]=qLArm[1] - 5*math.pi/180;    
  elseif byte==string.byte("s") then  
    qLArm[2]=qLArm[2] - 5*math.pi/180;    
  elseif byte==string.byte("d") then  
    qLArm[3]=qLArm[3] - 5*math.pi/180;    
  elseif byte==string.byte("f") then  
    qLArm[4]=qLArm[4] - 5*math.pi/180;    
  elseif byte==string.byte("g") then  
    qLArm[5]=qLArm[5] - 5*math.pi/180;    
  elseif byte==string.byte("h") then  
    qLArm[6]=qLArm[6] - 5*math.pi/180;    

  elseif byte==string.byte("b") then  
    check_kinematics();

  elseif byte==string.byte("k") then  
    qLArm={0,0,0,0,0,0};
  end

  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);



  return true
 
end

function update()
  count = count + 1;

  walk.active = false;
  walk.upper_body_override_on();

  -- Update State Machines 
  Motion.update();
  Body.update();
  lcount = lcount + 1;
  if (count ~= lcount) then
    print('count: '..count)
    print('lcount: '..lcount)
    Speak.talk('missed cycle');
    lcount = count;
  end
  io.stdout:flush();  
end

-- Initialize
Motion.entry()
count = 0;
lcount = 0;
tUpdate = unix.time();
Motion.event("standup");

-- if using Webots simulator just run update
local tDelay = 0.005 * 1E6; -- Loop every 5ms


qLArm = Body.get_larm_position();
qRArm = Body.get_rarm_position();

--qLArm = vector.new({90,0,0,-90,-60,0})*math.pi/180;
--qRArm = vector.new({90,0,0,-90,60,0})*math.pi/180;


while (true) do
	-- Run Updates
  process_keyinput();
  update();

  -- Debug Messages every 1 second
  t_diff = Body.get_time() - (t_last or 0);
  if(t_diff>1) then
		--print('qLArm',qLArm)
    t_last = Body.get_time();
  end

  if(darwin) then
    unix.usleep(tDelay);
  end
end
