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

qLArm0 = vector.new({90,0,0,0,0,0})*math.pi/180;
qRArm0 = vector.new({90,0,0,0,0,0})*math.pi/180;

--Arm pose for qual4
qLArm1 = vector.new({71,81,18,-87,-90,30})*math.pi/180;
qRArm1 = vector.new({71,-81,-18,-87,90,-30})*math.pi/180;



bodyHeight = walk.bodyHeight;
bodyTilt = walk.bodyTilt;
footX = walk.footX;
footY = walk.footY;
supportX = walk.supportX;


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
      qLArm =  (1-ph)*qLArm0 + qLArm1 * ph;
      qRArm =  (1-ph)*qRArm0 +  qRArm1 * ph;
 
      Body.set_l_gripper_command({math.pi/4,-math.pi/4});
      Body.set_r_gripper_command({math.pi/4,-math.pi/4});

    else
      qLArm = qLArm1;
      qRArm = qRArm1;

      arm_initing = 0;
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

    end   

    Body.set_larm_command(qLArm);
    Body.set_rarm_command(qRArm);
  end
  Body.set_head_command({0,60*math.pi/180});
end

current_action = 0;


--ArmX: 0.48 
--ArmY: 0.50
--Fully raised arm: 0.30
--Lowered arm height : 0


action_start_time = 0;
action_duration = 0;
current_action=0;




cycle = 0;

action_mount_ladder =  {
  --Attach left arm
  {
   {0.41,0.50,0.25,0};   --Raise Left arm 
   null,null,null,
   0.5, --duration
  },
  {
   {0.49,0.50,0.27,0};   --Extend
   null,null,null,
   0.5, --duration
  },
  {
   {0.49,0.50,0.27,1};   --Grip
   null,null,null,
   0.1, --duration
  },

  --Attach right arm
  {
   null, 
   {0.41,-0.50,0.25,0};   --Raise Left arm 
   null, null, 
   0.5, --duration
  },
  {
   null, 
   {0.49,-0.50,0.27,0};   --Extend
   null, null, 
   0.5, --duration
  },
  {
   null, 
   {0.49,-0.50,0.27,1};   --Grip
   null, null, 
   0.1, --duration
  },

  --Place left foot
  {
    null,null,
    {0,0.10,0.35,0}, 
    null,
    0.5,
  },
  {
    null,null,
    {0.48,0.18,0.35,-0.26}, --Tilt
    null,
    0.5,
  },
  {
    null,null,
    {0.48,0.18,0.33,-0.26},
    null,
    0.2,
  },

  --Place right foot
  {
    null,null,
    null,
    {0,-0.10,0.35,0},
    0.5,
  },
  {
    null,null,
    null,
    {0.48,-0.18,0.35,-0.26},
    0.5,
  },
  {
    null,null,
    null,
    {0.48,-0.18,0.33,-0.26},
    0.2,
  },
}


action_llarm_up = {
  {
   {0.41,0.50,-0.03,0};   --Left arm retract, gripper open
   null, --Right arm not moving
   null, --LLeg not movin
   null, --RLeg not movin
   0.2, --duration
  },
  {
   {0.41,0.50,0.25,0};   --Raise Left arm 
   null,null,null,
   0.7, --duration
  },
  {
   {0.50,0.50,0.27,0};   --Extend
   null,null,null,
   0.7, --duration
  },
  {
   {0.50,0.50,0.27,1};   --Grip
   null,null,null,
   0.1, --duration
  },
}

action_rlarm_up = {
  {
   null, --Left arm not moving
   {0.41,-0.50,-0.03,0};   --R arm retract, gripper open
   null, null, 
   0.2, --duration
  },
  {
   null, 
   {0.41,-0.50,0.25,0};   --Raise Left arm 
   null, null, 
   0.7, --duration
  },
  {
   null, 
   {0.50,-0.50,0.27,0};   --Extend
   null, null, 
   0.7, --duration
  },
  {
   null, 
   {0.50,-0.50,0.27,1};   --Grip
   null, null, 
   0.1, --duration
  },
}

action_legs_up = {

  {  --Pull body 2ocm higher
    {0.48, 0.50,0.07,1}, 
    {0.48,-0.50,0.07,1},
    {0.48, 0.18,0.13,-0.26}, 
    {0.48,-0.18,0.13,-0.26},
    1.0,
  },
  {  --Lift right foot
    null,null,null,
    {0.0, -0.18, 0.16,-0.26},
     0.2,
  },
  {  --Raise right foot
    null,null,null,
    {0.48, -0.18, 0.49,-0.26}, 
     0.5,
  },
  {  --Land right foot
    null,null,null,
    {0.48, -0.18, 0.43,-0.26},
     0.2,
  },

  { --Lift left foot
    null,null,
    {0.0, 0.18, 0.16,-0.26}, 
    null,
    0.5,
  },
  { --Raise left foot
    null,null,
    {0.48, 0.18, 0.49,-0.26}, 
    null,
    0.5,
  },
  { --Raise left foot
    null,null,
    {0.48, 0.18, 0.43,-0.26}, 
    null,
    0.2,
  },
  {  --Pull body 1ocm higher
    {0.48, 0.50,-0.03,1}, 
    {0.48,-0.50,-0.03,1},
    {0.48, 0.39,0.33,-0.26}, --Wider stance
    {0.48,-0.39,0.33,-0.26},
    1.0,
  },
}


LLeg={0,footY,0,0};
RLeg={0,-footY,0,0};

function auto_move_arms()
  t = Body.get_time();
  if is_moving==0 then 
    action_start_time = t;
    action_duration = 0;
    return;
  end

  if is_moving==1 then --Start moving
    LArmTarget1 = {trLArm[1],trLArm[2],trLArm[3]};
    RArmTarget1 = {trRArm[1],trRArm[2],trRArm[3]};
    LLegTarget1 = {LLeg[1],LLeg[2],LLeg[3],LLeg[4]};
    RLegTarget1 = {RLeg[1],RLeg[2],RLeg[3],LLeg[4]};
    is_moving = 2;
    action_count = 0;
    action_duration = 0;
    action_start_time = t;
    return;
  end

  if (t>action_start_time + action_duration) then --New motion
    action_count = action_count + 1;
    action_start_time = action_start_time + action_duration;

    if action_count > #current_action then
      is_moving = 0;
      return;
    end

    LArmTarget0 = LArmTarget1;
    RArmTarget0 = RArmTarget1;
    LLegTarget0 = LLegTarget1;
    RLegTarget0 = RLegTarget1;


    current_frame = current_action[action_count];
    LArmTargetF = current_frame[1];    
    RArmTargetF = current_frame[2];    
    LLegTargetF = current_frame[3];    
    RLegTargetF = current_frame[4];    
    action_duration = current_frame[5];

    if LArmTargetF then
      LArmTarget1 = {LArmTargetF[1],LArmTargetF[2],LArmTargetF[3]};
      if LArmTargetF[4]>0 then
        Body.set_l_gripper_command({0,0});--Grip
      else
        Body.set_l_gripper_command({math.pi/4,-math.pi/4});--Release 
      end
    else
      LArmTarget1 = LArmTarget0;
    end

    if RArmTargetF then
      RArmTarget1 = {RArmTargetF[1],RArmTargetF[2],RArmTargetF[3]};
      if RArmTargetF[4]>0 then
        Body.set_r_gripper_command({0,0});--Grip
      else
        Body.set_r_gripper_command({math.pi/4,-math.pi/4});--Release 
      end
    else
      RArmTarget1 = RArmTarget0;
    end

    if LLegTargetF then
      LLegTarget1 = {
	LLegTargetF[1],LLegTargetF[2],LLegTargetF[3],LLegTargetF[4]};
    else
      LLegTarget1 = LLegTarget0;
    end

    if RLegTargetF then
      RLegTarget1 = {
	RLegTargetF[1],RLegTargetF[2],RLegTargetF[3],RLegTargetF[4]};
    else
      RLegTarget1 = RLegTarget0;
    end
  end

  ph = (t-action_start_time)/action_duration;

--print("ph:",ph)

  trLArm[1] = (1-ph)*LArmTarget0[1] + ph*LArmTarget1[1];      
  trLArm[2] = (1-ph)*LArmTarget0[2] + ph*LArmTarget1[2];      
  trLArm[3] = (1-ph)*LArmTarget0[3] + ph*LArmTarget1[3];      

  trRArm[1] = (1-ph)*RArmTarget0[1] + ph*RArmTarget1[1];      
  trRArm[2] = (1-ph)*RArmTarget0[2] + ph*RArmTarget1[2];      
  trRArm[3] = (1-ph)*RArmTarget0[3] + ph*RArmTarget1[3];      

  LLeg[1] = (1-ph)*LLegTarget0[1] + ph*LLegTarget1[1];
  LLeg[2] = (1-ph)*LLegTarget0[2] + ph*LLegTarget1[2];
  LLeg[3] = (1-ph)*LLegTarget0[3] + ph*LLegTarget1[3];
  LLeg[4] = (1-ph)*LLegTarget0[4] + ph*LLegTarget1[4];

  RLeg[1] = (1-ph)*RLegTarget0[1] + ph*RLegTarget1[1];
  RLeg[2] = (1-ph)*RLegTarget0[2] + ph*RLegTarget1[2];
  RLeg[3] = (1-ph)*RLegTarget0[3] + ph*RLegTarget1[3];
  RLeg[4] = (1-ph)*RLegTarget0[4] + ph*RLegTarget1[4];
  

  pTorso = vector.new({supportX-footX, 0,bodyHeight,0,bodyTilt,0});
  pLLeg = {LLeg[1], LLeg[2], LLeg[3],0,LLeg[4],0};
  pRLeg = {RLeg[1], RLeg[2], RLeg[3],0,RLeg[4],0};

  motion_arms_ik();

  q = Kinematics.inverse_legs(pLLeg, pRLeg, pTorso, 0);

  Body.set_body_hardness(1);
  Body.set_lleg_command(q);

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

    qLArmInv[5] = util.mod_angle(qLArmInv[5]);
    qRArmInv[5] = util.mod_angle(qRArmInv[5]);

    Body.set_larm_command(qLArmInv);
    Body.set_rarm_command(qRArmInv);

      trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6]=
      trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6];

      trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6]=
      trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6];


  else

--    is_moving=0;
    print("STUCK!")

      trLArm[1],trLArm[2],trLArm[3],trLArm[4],trLArm[5],trLArm[6]=
      trLArmOld[1],trLArmOld[2],trLArmOld[3],trLArmOld[4],trLArmOld[5],trLArmOld[6];

      trRArm[1],trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6]=
      trRArmOld[1],trRArmOld[2],trRArmOld[3],trRArmOld[4],trRArmOld[5],trRArmOld[6];
  end

end

function start_action(var)
  if is_moving==0 then
    is_moving = 1;
    current_action = var;  
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
    start_action(action_mount_ladder);
  elseif byte==string.byte("2") then  
    start_action(action_legs_up);

  elseif byte==string.byte("3") then  
    start_action(action_llarm_up);

  elseif byte==string.byte("4") then  
    start_action(action_rlarm_up);

  elseif byte==string.byte("5") then  

  elseif byte==string.byte("6") then  
  elseif byte==string.byte("7") then  

  elseif byte==string.byte("8") then  
  elseif byte==string.byte("9") then  
  elseif byte==string.byte("0") then  



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
  elseif byte==string.byte("c") then  
    trRArm[4]=trRArm[4]+0.1;
    update_arm = true;
  elseif byte==string.byte("v") then  
    trRArm[4]=trRArm[4]-0.1;
    update_arm = true;





  elseif byte==string.byte("t") then  --Open gripper
    Body.set_r_gripper_command({math.pi/6,-math.pi/6});
  elseif byte==string.byte("y") then  --Close gripper
    Body.set_r_gripper_command({0,0});

  elseif byte==string.byte("g") then  --Move to the object
   is_moving=1;
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
  if arm_initing>0 then
    Motion.update();
  end
  Body.update();
  auto_move_arms();

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
