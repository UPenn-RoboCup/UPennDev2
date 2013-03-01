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


--Arm target transforms

--[[
trLArmOld = vector.new({0.16, 0.24, -0.09, 0,0,0});
trRArmOld = vector.new({0.16, -0.24, -0.09, 0,0,0});

trLArmOld = vector.new({0.16, 0.24, -0.07, 0, 0, -math.pi/4});
trRArmOld = vector.new({0.16, -0.24, -0.07, 0, 0, math.pi/4});
--]]

--New position considering the hand offset
trLArmOld = vector.new({0.28, 0.22, 0.05, 0, 0, -math.pi/2});
trRArmOld = vector.new({0.28, -0.22,0.05, 0, 0, math.pi/2});


trLArm=vector.new({0,0,0,0,0,0});
trRArm=vector.new({0,0,0,0,0,0});
trLArm0=vector.new({0,0,0,0,0,0});
trRArm0=vector.new({0,0,0,0,0,0});

trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6]=
trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

trLArm0[1],trLArm0[2],trLArm0[3],trLArm0[4],trLArm0[5],trLArm0[6]=
trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6]=
trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];

trRArm0[1],trRArm0[2],trRArm0[3],trRArm0[4],trRArm0[5],trRArm0[6]=
trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];

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



arm_initing = 1;
arm_init_stage = 0;
arm_init_t0 = 0;
arm_stage_duration={2.0,1.0,1.0,1.0};
is_moving = 0;

function init_arms()
  t = Body.get_time();
  if arm_initing>0 then     
    walk.upper_body_override_on();
    current_duration = arm_stage_duration[arm_init_stage+1];
    if arm_init_t0+current_duration< t then
      arm_init_stage = arm_init_stage+1;
      arm_init_t0 = t;
    end
    ph = (t-arm_init_t0) /current_duration;
    if arm_init_stage==0 then
      qLArm =  {math.pi/2, math.pi/2 * ph, 0, 0,0,0};
      qRArm =  {math.pi/2, -math.pi/2 * ph, 0, 0,0,0};      
    elseif arm_init_stage==1 then

      qLArm =  {(1-ph)*math.pi/2, math.pi/2 , ph*math.pi/2,  
			0,-math.pi/2*ph,0};
      qRArm =  {(1-ph)*math.pi/2, -math.pi/2,  -ph*math.pi/2, 
			0,math.pi/2*ph,0};      

    elseif arm_init_stage==2 then
      qLArm =  {0, math.pi/2-math.pi/9*ph , math.pi/2,
		 (math.pi/9-math.pi/2)*ph, -math.pi/2,-math.pi/2*ph};
      qRArm =  {0, -math.pi/2+ math.pi/9*ph , -math.pi/2,
		  (math.pi/9-math.pi/2)*ph, math.pi/2,math.pi/2*ph};      
    else
      qLArm =  {0, math.pi/2 -math.pi/9 ,math.pi/2,
		 (math.pi/9-math.pi/2), -math.pi/2,-math.pi/2};
      qRArm =  {0, -math.pi/2+ math.pi/9 ,-math.pi/2,
		  (math.pi/9-math.pi/2), math.pi/2, math.pi/2};      

      arm_initing = 0;
      trLArmOld = Kinematics.l_arm_torso(qLArm);
      trRArmOld = Kinematics.r_arm_torso(qRArm);

      --Lower the hand a bit to avoid singularity
      trLArmOld[3]=trLArmOld[3] - 0.02; 
      trRArmOld[3]=trRArmOld[3] - 0.02;

      trLArmOld[6]= - math.pi/2; 
      trRArmOld[6]= math.pi/2;

      trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6]=
	trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

      trLArm0[1],trLArm0[2],trLArm0[3],trLArm0[4],trLArm0[5],trLArm0[6]=
	trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

      trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6]=
	trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];

      trRArm0[1],trRArm0[2],trRArm0[3],trRArm0[4],trRArm0[5],trRArm0[6]=
	trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];

    end   

    Body.set_larm_command(qLArm);
    Body.set_rarm_command(qRArm);
--    walk.upper_body_override(qLArm, qRArm, walk.bodyRot0);

  end
  Body.set_head_command({0,60*math.pi/180});

end


turnAngle = 0;
radius = 0.2;


function calculate_arm_position()
  --We assume that the wheel is centered at (0.3,0,1.2) and has radius 0.2

   local handle_pos = {0.3,0,1.2};

   trGripL = Transform.eye()
       * Transform.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * Transform.rotX(turnAngle*rotating_direction)
       * Transform.trans(0,radius,0)
       * Transform.rotZ(-math.pi/2);

   trGripR = Transform.eye()
       * Transform.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * Transform.rotX(turnAngle*rotating_direction)
       * Transform.trans(0,-radius,0)
       * Transform.rotZ(math.pi/2);

   body_pos = Body.get_sensor_gps();
   body_rpy = Body.get_sensor_imuAngle();

   body_pos = {0.02,0,1.145};
   body_rpy = {0,0,0};


   trBody=Transform.eye()
   * Transform.trans(body_pos[1],body_pos[2],body_pos[3])
   * Transform.rotZ(body_rpy[3])
   * Transform.rotY(body_rpy[2]);

   trLArm = Transform.position6D(Transform.inv(trBody)*trGripL);
   trRArm = Transform.position6D(Transform.inv(trBody)*trGripR);

   motion_arms_ik();

--print("Angle radius:",turnAngle*180/math.pi, radius)
end













function update_cognition()

  body_pos = Body.get_sensor_gps();
  body_rpy = Body.get_sensor_imuAngle();
  object_pos = wcm.get_robot_gps_ball();

  trBody=Transform.eye()
   * Transform.trans(body_pos[1],body_pos[2],body_pos[3])
   * Transform.rotZ(body_rpy[3])
   * Transform.rotY(body_rpy[2]);

  trEffector = Transform.eye()
   * Transform.trans(object_pos[1],object_pos[2],object_pos[3])
   * Transform.rotZ(trLArm[6]); --End effector transform (currently yaw only)

  trRelative = Transform.inv(trBody)*trEffector;
  pRelative = {trRelative[1][4],trRelative[2][4],trRelative[3][4]};


--[[
  print("body abs pos:",unpack( body_pos )); --Check Body GPS 
  --print("body pitch and yaw:",body_rpy[2]*180/math.pi, body_rpy[3]*180/math.pi);
  print("object abs pos:",unpack( object_pos )); 

  print("object rel pos:",unpack(pRelative))

  print("current LArm rel pos:",
	trLArm[1],trLArm[2],trLArm[3]);
--]]

end


rotating_direction = 1;


function auto_move_arms()
  if is_moving==0 then return;
  end
  
  velATurn = 1*math.pi/180;
  velR = 0.001;


  radius0 = 0.23;
  radius1 = 0.30;
  angle0 = 23*math.pi/180;
  angle1 = -23*math.pi/180;

  nTurn = 8;

  if is_moving == 2*nTurn+1 then --Finished rotating
    if radius < radius1 then radius = radius + velR; end    
    if turnAngle <0 then turnAngle = turnAngle + velATurn;end
    if radius>=radius1 and turnAngle>=0 then
      is_moving = 0; --Finish this
    end
    calculate_arm_position();
   
    return;
  end



  if is_moving%2==1 then 
    if radius > radius0 then radius = radius - velR; 
    elseif turnAngle<angle0 then 
      turnAngle = turnAngle + velATurn;
    else
      --Close gripper
      Body.set_l_gripper_command({0,0});
      Body.set_r_gripper_command({0,0});
      is_moving = is_moving + 1;
    end
  elseif is_moving%2==0 then
    --rotate
    if turnAngle >angle1 then
      turnAngle = turnAngle - velATurn;
    else
      is_moving = is_moving + 1;
      --Open gripper 
      Body.set_l_gripper_command({math.pi/4,-math.pi/4});
      Body.set_r_gripper_command({math.pi/4,-math.pi/4});
    end
  end

  calculate_arm_position();

  


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




function motion_arms_ik()
  if arm_initing>0 then return; end
  qLArmInv, dist1 = check_ik(trLArm, 1);
  qRArmInv, dist2 = check_ik(trRArm, 0);

--  print("Error:",dist1)


  if dist1<0.01 and dist2<0.01 then
--  if true then

    walk.upper_body_override_on();
--      walk.upper_body_override(qLArmInv, qRArmInv, walk.bodyRot0);

--HACK: WRIST ANGLE INVERSION

    qLArmInv[5] = util.mod_angle(qLArmInv[5]);
    qRArmInv[5] = util.mod_angle(qRArmInv[5]);

    Body.set_larm_command(qLArmInv);
    Body.set_rarm_command(qRArmInv);

      trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6]=
      trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6];

      trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6]=
      trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6];
  else

    is_moving=0;
    print("STUCK!")


      trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6]=
      trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

      trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6]=
      trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];
  end

--  print("LArm transform:",unpack(trLArm))

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

  --Arm target position control

  if byte==string.byte("j") then  
    turnAngle = turnAngle + 5*math.pi/180;
    calculate_arm_position();

  elseif byte==string.byte("l") then  
    turnAngle = turnAngle - 5*math.pi/180;
    calculate_arm_position();

  elseif byte==string.byte("k") then  
    turnAngle = 0;
    radius = 0.25;
    calculate_arm_position();

  elseif byte==string.byte("i") then  
    radius = radius + 0.01;
    calculate_arm_position();

  elseif byte==string.byte(",") then  
    radius = radius - 0.01;
    calculate_arm_position();

  elseif byte==string.byte("e") then  --Open gripper
    Body.set_l_gripper_command({math.pi/6,-math.pi/6});
    Body.set_r_gripper_command({math.pi/6,-math.pi/6});
    update_arm = true;

  elseif byte==string.byte("r") then  --Close gripper
    Body.set_l_gripper_command({0,0});
    Body.set_r_gripper_command({0,0});
    update_arm = true;




  elseif byte==string.byte("f") then  --Start turning
    --open gripper
    Body.set_l_gripper_command({math.pi/4,-math.pi/4});
    Body.set_r_gripper_command({math.pi/4,-math.pi/4});
    is_moving = 1;
    rotating_direction = 1;
    


  elseif byte==string.byte("g") then  --Start turning
    --open gripper
    Body.set_l_gripper_command({math.pi/4,-math.pi/4});
    Body.set_r_gripper_command({math.pi/4,-math.pi/4});
    is_moving = 1;
    rotating_direction = -1;









  elseif byte==string.byte("s") then  
    trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6]=
    trLArm0[1],trLArm0[2],trLArm0[3],trLArm0[4],trLArm0[5],trLArm0[6];
    trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6]=
    trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6];
    update_arm = true;

  elseif byte==string.byte("w") then  
    trLArm[1]=trLArm[1]+0.01;
    update_arm = true;
  elseif byte==string.byte("x") then  
    trLArm[1]=trLArm[1]-0.01;
    update_arm = true;
  elseif byte==string.byte("a") then  
    trLArm[2]=trLArm[2]+0.01;
    update_arm = true;
  elseif byte==string.byte("d") then  
    trLArm[2]=trLArm[2]-0.01;
    update_arm = true;
  elseif byte==string.byte("q") then  
    trLArm[3]=trLArm[3]+0.01;
    update_arm = true;
  elseif byte==string.byte("z") then  
    trLArm[3]=trLArm[3]-0.01;
    update_arm = true;
  end

  if ( update_arm ) then  
   is_moving=0;
   motion_arms_ik();     
  end
  return true
 
end

function update()
  count = count + 1;

  walk.active = false;

  -- Update State Machines 
  Motion.update();
  Body.update();
  update_cognition();
  auto_move_arms();

  -- Check if the last update completed without errors
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

--calculate_arm_space();

arm_initing = 1;
arm_init_t0 = Body.get_time();


while (true) do
	-- Run Updates
  process_keyinput();

--  arm_demo();
  Team.update();
  init_arms();

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
