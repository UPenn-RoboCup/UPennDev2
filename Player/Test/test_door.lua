


-- CHARLI laser testing
print('Testing ARMS')

cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
require('init')

require 'carray'

require('Config')

--HACK to change the body height
Config.walk.bodyHeight = 1.00;




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

if (string.find(Config.platform.name,'THOROP')) then
  thorop=true;
else
  thorop=false;
end



--Arm target transforms

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

torsoYaw = 0;

Body.set_l_gripper_command({0,0});
Body.set_r_gripper_command({0,0});
Body.set_l_gripper_hardness({1,1});
Body.set_r_gripper_hardness({1,1});
Body.set_waist_command(0);
Body.set_waist_hardness(1);

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


is_moving = 0;

--THOR VALUES
arm_init_motion_thorop={
  {
    vector.new({90,0,0,0,0,0})*math.pi/180,
    vector.new({90,0,0,0,0,0})*math.pi/180,
    1.0,
  },
  {
    vector.new({0,45,90,-90,-90,0})*math.pi/180,
    vector.new({90,-45,0,-90,-225,45})*math.pi/180,
    1.0,
  },
}














arm_init_motion_atlas={
  {
    vector.new({90,30,-90,0, -90,0})*math.pi/180,
    vector.new({90,-30,90,0, -90,0})*math.pi/180,
    0.2,
  },
  {
    vector.new({90,30,0,0, -60,0})*math.pi/180,
    vector.new({90,-30,0,0, -60,0})*math.pi/180,
    1.0,
  },
--[[
  {
    vector.new({90,0,0,-90,-60,-90})*math.pi/180,
    vector.new({90,0,0,-90,60,90})*math.pi/180,
    1.0,
  },
--]]

  --VERTICAL HAND POSTURE (WORKS BETTER)
  {
    vector.new({90,30,0,-90,0,-90})*math.pi/180,
    vector.new({90,-30,0,-90,0,90})*math.pi/180,
    1.0,
  },


}















if thorop then
  arm_init_motion = arm_init_motion_thorop;
else
  arm_init_motion = arm_init_motion_atlas;
end


arm_init_count = 1; --start arm initing
arm_init_t0 = Body.get_time();
qLArm0=vector.new(Body.get_larm_position());
qRArm0=vector.new(Body.get_rarm_position());


function init_arms()
  walk.upper_body_override_on();
  Body.set_head_command({0,60*math.pi/180});

  t = Body.get_time();
  if arm_init_count==0 then return;
  elseif arm_init_count>#arm_init_motion then

    trLArmOld = Kinematics.l_arm_torso(qLArm);
    trRArmOld = Kinematics.r_arm_torso(qRArm);

    trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6]=
        trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

    trLArm0[1],trLArm0[2],trLArm0[3],trLArm0[4],trLArm0[5],trLArm0[6]=
	trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

    trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6]=
	trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];

    trRArm0[1],trRArm0[2],trRArm0[3],trRArm0[4],trRArm0[5],trRArm0[6]=
	trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];
    arm_init_count = 0;


print("TRARM0 RPY:",180*trRArm0[4]/math.pi,180*trRArm0[5]/math.pi,180*trRArm0[6]/math.pi);

    return;
  end

  current_duration = arm_init_motion[arm_init_count][3];
  if t>arm_init_t0+current_duration then
    qLArm0 = arm_init_motion[arm_init_count][1];
    qRArm0 = arm_init_motion[arm_init_count][2];
    arm_init_t0 = arm_init_t0 + current_duration;
    arm_init_count = arm_init_count+1;
    return;
  end

  ph = (t-arm_init_t0) /current_duration;

  qLArm = (1-ph) * qLArm0 + ph * arm_init_motion[arm_init_count][1];
  qRArm = (1-ph) * qRArm0 + ph * arm_init_motion[arm_init_count][2];
  Body.set_larm_command(qLArm);
  Body.set_rarm_command(qRArm);
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



--rArm trajectories
trRTarget1 = {0.435, -0.3777, 0.07366, trRArm0[4], trRArm0[5], trRArm0[6]};
trRTarget2 = {0.435, -0.3277, 0.07366, trRArm0[4], trRArm0[5], trRArm0[6]}; 
trRTarget3 = {0.435, -0.3277, 0.04366, trRArm0[4], trRArm0[5], trRArm0[6]}; 
trRTarget4 = {0.335, -0.3277, 0.04366, trRArm0[4], trRArm0[5], trRArm0[6]}; 
trRTarget5 = {0.335, -0.3277, -0.030, trRArm0[4], trRArm0[5], trRArm0[6]}; 
trRTarget6 = {0.10, -0.3277, -0.13, trRArm0[4], trRArm0[5], trRArm0[6]}; 



trLTarget1 = {0.22, -0.15, 0.14, trLArm0[4],trLArm0[5],trLArm0[6]};
trLTarget2 = {0.22, -0.06, 0.14, trLArm0[4],trLArm0[5],trLArm0[6]};
trLTarget3 = {0.34, -0.06, 0.14, trLArm0[4],trLArm0[5],trLArm0[6]};
trLTarget4 = {0.34, -0.14, 0.14, trLArm0[4],trLArm0[5],trLArm0[6]};


dTorsoYaw = 0.05*math.pi/180;
dTorsoYaw = 0.2*math.pi/180;


vel1=0.002;
vel2= 0.0005;


function auto_move_arms()
  if is_moving==0 then return;
  end
  if is_moving==1 then
    Body.set_r_gripper_command({0,0});

    pRelative = trRTarget1;
    move_type = 0; --Linear movement
  elseif is_moving==2 then
    pRelative = trRTarget2;
    move_type = 0; --Linear movement
    knobAngle1 = 0;
  elseif is_moving==3 then --Grip and rotate the knob
    Body.set_r_gripper_command({-math.pi/6,math.pi/6});
    pRelative = trRTarget3;
    move_type = 0; --Linear movement
  elseif is_moving==4 then --Grip and rotate the knob
    pRelative = trRTarget4;
    move_type = 0; --Linear movement
    knobAngle1 = 0;
  elseif is_moving==5 then 
    Body.set_r_gripper_command({0,0});
    pRelative = trRTarget5;
    move_type = 0; --Linear movement
  elseif is_moving==6 then 
    Body.set_r_gripper_command({0,0});
    pRelative = trRTarget6;
    move_type = 0; --Linear movement
    knobAngle1 = 0;
  elseif is_moving==7 then --Turn torso slowly
    torsoYaw = torsoYaw - dTorsoYaw;
    Body.set_waist_command(torsoYaw);
    if torsoYaw<-35*math.pi/180 then
      is_moving = 8;      
    end
    move_type = 2;
  elseif is_moving==8 then 
    pRelative = trLTarget1;
    move_type=1;
  elseif is_moving==9 then 
    pRelative = trLTarget2;
  elseif is_moving==10 then 
    pRelative = trLTarget3;
  elseif is_moving==11 then 
    pRelative = trLTarget4;

  elseif is_moving==12 then
    torsoYaw = torsoYaw - dTorsoYaw;
    Body.set_waist_command(torsoYaw);
    if torsoYaw<-90*math.pi/180 then
      is_moving = 0;      
      return;
    end
    move_type = 2;
  end


  if move_type==0 then --Linear movement, right arm
    qInv, dist = check_ik(pRelative,0); --RArm
    armDir = vector.new({
	pRelative[1]-trRArm[1],
	pRelative[2]-trRArm[2],
	pRelative[3]-trRArm[3]});
    dRelative = math.sqrt(armDir[1]^2 + armDir[2]^2+armDir[3]^2);

    vel=vel1;
    if dRelative<0.02 then vel=vel2;  end

    if dRelative<0.0025 then --Approached the target
      print("TARGET REACHED")
      is_moving = is_moving + 1;
    end
    trRArm[1] = trRArm[1] + vel*armDir[1]/dRelative;
    trRArm[2] = trRArm[2] + vel*armDir[2]/dRelative;
    trRArm[3] = trRArm[3] + vel*armDir[3]/dRelative;
    motion_arms_ik();  
  elseif move_type==1 then --Linear movement, left arm
    qInv, dist = check_ik(pRelative,1); --LArm
    armDir = vector.new({
	pRelative[1]-trLArm[1],
	pRelative[2]-trLArm[2],
	pRelative[3]-trLArm[3]});
    dRelative = math.sqrt(armDir[1]^2 + armDir[2]^2+armDir[3]^2);

    vel=vel1;
    if dRelative<0.02 then vel=vel2;  end

    if dRelative<0.0025 then --Approached the target
      print("TARGET REACHED")
      is_moving = is_moving + 1;
    end
    trLArm[1] = trLArm[1] + vel*armDir[1]/dRelative;
    trLArm[2] = trLArm[2] + vel*armDir[2]/dRelative;
    trLArm[3] = trLArm[3] + vel*armDir[3]/dRelative;
    motion_arms_ik();  
  end
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
  if arm_init_count>0 then return; end
  qLArmInv, dist1 = check_ik(trLArm, 1);
  qRArmInv, dist2 = check_ik(trRArm, 0);

--  print("Error:",dist1)


  if dist1<0.01 and dist2<0.01 then
--  if true then

      walk.upper_body_override_on();
--      walk.upper_body_override(qLArmInv, qRArmInv, walk.bodyRot0);


--print("ryaw:",qRArmInv[5]*180/math.pi)

--Problem: wrist yaw go from -181 to -180
    qLArmInv[5] = util.mod_angle(qLArmInv[5]+math.pi)-math.pi;
    qRArmInv[5] = util.mod_angle(qRArmInv[5]+math.pi)-math.pi;

--print("ryaw2:",qRArmInv[5]*180/math.pi)

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
  if byte==string.byte("s") then  
    trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6]=
    trLArm0[1],trLArm0[2],trLArm0[3],trLArm0[4],trLArm0[5],trLArm0[6];

    trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6]=
    trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6];

    update_arm = true;

  elseif byte==string.byte("k") then  
    trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6]=
    trRArm0[1],trRArm0[2],trRArm0[3],trRArm0[4],trRArm0[5],trRArm0[6];

    trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6]=
    trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6];

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



  elseif byte==string.byte("1") then  
    trRArm[6]=trRArm[6]+0.1;
    update_arm = true;
  elseif byte==string.byte("2") then  
    trRArm[6]=trRArm[6]-0.1;
    update_arm = true;
  elseif byte==string.byte("3") then  
    trRArm[4]=trRArm[4]+0.1;
    update_arm = true;
  elseif byte==string.byte("4") then  
    trRArm[4]=trRArm[4]-0.1;
    update_arm = true;
  elseif byte==string.byte("5") then  
    trRArm[5]=trRArm[5]+0.1;
    update_arm = true;
  elseif byte==string.byte("6") then  
    trRArm[5]=trRArm[5]-0.1;
    update_arm = true;


  elseif byte==string.byte("9") then  
    torsoYaw = torsoYaw + 0.1;
    Body.set_waist_command(torsoYaw);
  elseif byte==string.byte("0") then  
    torsoYaw = torsoYaw - 0.1;
    Body.set_waist_command(torsoYaw);




  elseif byte==string.byte("e") then  --Open gripper
    Body.set_l_gripper_command({math.pi/6,-math.pi/6});
  elseif byte==string.byte("r") then  --Close gripper
    Body.set_l_gripper_command({0,0});

  elseif byte==string.byte("i") then  
    trRArm[1]=trRArm[1]+0.01;
    update_arm = true;
  elseif byte==string.byte(",") then  
    trRArm[1]=trRArm[1]-0.01;
    update_arm = true;
  elseif byte==string.byte("j") then  
    trRArm[2]=trRArm[2]+0.01;
    update_arm = true;
  elseif byte==string.byte("l") then  
    trRArm[2]=trRArm[2]-0.01;
    update_arm = true;
  elseif byte==string.byte("u") then  
    trRArm[3]=trRArm[3]+0.01;
    update_arm = true;
  elseif byte==string.byte("m") then  
    trRArm[3]=trRArm[3]-0.01;
    update_arm = true;
  elseif byte==string.byte("b") then  
    trRArm[6]=trRArm[6]+0.1;
    update_arm = true;
  elseif byte==string.byte("n") then  
    trRArm[6]=trRArm[6]-0.1;
    update_arm = true;

  elseif byte==string.byte("t") then  --Open gripper
    Body.set_r_gripper_command({math.pi/6,-math.pi/6});
  elseif byte==string.byte("y") then  --Close gripper
    Body.set_r_gripper_command({0,0});

  elseif byte==string.byte("g") then  --Move to the object
   is_moving=1;

  elseif byte==string.byte("/") then     
    print("TRLArm:",trLArm[1],trLArm[2],trLArm[3]);
    print("TRRArm:",trRArm[1],trRArm[2],trRArm[3]);
  end

  if ( update_arm ) then  
   is_moving=0;
   motion_arms_ik();  
  end
  return true
 
end



walk.upper_body_override_on();


function update()
  count = count + 1;

  walk.active = false;

  -- Update State Machines 
  Motion.update();
  Body.update();
  update_cognition();
  auto_move_arms();
  Team.update();
--[[	
	-- Update the laser scanner
	lidar_scan = WebotsLaser.get_scan()
	-- Set the Range Comm Manager values
	rcm.set_lidar_ranges( carray.pointer(lidar_scan) );
	rcm.set_lidar_timestamp( Body.get_time() )
	--print("Laser data:",unpack(rcm.get_lidar_ranges()))
	
	-- Show the odometry
	odom, odom0 = mcm.get_odometry();
	rcm.set_robot_odom( vector.new(odom) )
	
	-- Show IMU
  imuAngle = Body.get_sensor_imuAngle();
	rcm.set_robot_imu( vector.new(imuAngle) )
	gyr = Body.get_sensor_imuGyrRPY();
	rcm.set_robot_gyro( vector.new(gyr) );
--]]


--print("Roll", gyr[1], "Pitch",gyr[2]);

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
