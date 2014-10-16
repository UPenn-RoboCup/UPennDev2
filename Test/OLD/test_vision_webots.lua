dofile('Run/include.lua')

cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
local init = require('init')

local Config = require('Config');
local smindex = 0;

package.path = cwd..'/BodyFSM/'..Config.fsm.body[smindex+1]..'/?.lua;'..package.path;
package.path = cwd..'/HeadFSM/'..Config.fsm.head[smindex+1]..'/?.lua;'..package.path;
package.path = cwd..'/GameFSM/'..Config.fsm.game..'/?.lua;'..package.path;

local shm = require('shm')
local Body = require('Body')
local vector = require('vector')

local BodyFSM = require('BodyFSM');
local HeadFSM = require('HeadFSM');
local Motion = require('Motion');
local walk = require('walk');
local HeadTransform = require('HeadTransform')
local Speak = require('Speak')
local Vision = require('Vision')
local World = require('World')
local Team = require('Team')
local util = require('util')
require('vcm')
require('wcm')
require('gcm')
require('ocm')
require('matcm')
--local behaviorObstacle = require('behaviorObstacle')

local darwin = false;
local webots = false;

-- Enable OP specific 
if(Config.platform.name == 'OP') then
  darwin = true;
end

-- Enable Webots specific
if (string.find(Config.platform.name,'Webots')) then
  webots = true;
end

if Config.vision.enable_freespace_detection == 1 then 
  local OccupancyMap = require('OccupancyMap')
  OccupancyMap.entry();
end

-- initialize state machines
HeadFSM.entry();
Motion.entry();
World.entry();
Vision.entry();

Body.set_head_hardness({0.4,0.4});
controller.wb_robot_keyboard_enable(100);

-- main loop
local headsm_running=0;
local bodysm_running=0;

local count = 0;
t0=Body.get_time();
local last_vision_update_time=t0;

headangle=vector.new({0,10*math.pi/180});
targetvel=vector.zeros(3);
vision_update_interval = 0.04; --25fps update

-- set game state to ready to stop particle filter initiation
gcm.set_game_state(1);

Motion.event("standup")

function process_keyinput()
  local str = controller.wb_robot_keyboard_get_key();
  if str>0 then
    byte = str;
    -- Webots only return captal letter number
    if byte>=65 and byte<=90 then
      byte = byte + 32;
    end
    --Turn the head around

    if byte==string.byte("w") then
      headsm_running=0;
      headangle[2]=headangle[2]-5*math.pi/180;
     print("Headangle:", headangle[2]*180/math.pi)
    elseif byte==string.byte("a") then
      headangle[1]=headangle[1]+5*math.pi/180;
      headsm_running=0;
     print("Headangle:", headangle[2]*180/math.pi)
    elseif byte==string.byte("s") then	
      headangle[1],headangle[2]=0,0;
      headsm_running=0;
     print("Headangle:", headangle[2]*180/math.pi)
    elseif byte==string.byte("d") then
      headangle[1]=headangle[1]-5*math.pi/180;
      headsm_running=0;
     print("Headangle:", headangle[2]*180/math.pi)
    elseif byte==string.byte("x") then	
      headangle[2]=headangle[2]+5*math.pi/180;
     print("Headangle:", headangle[2]*180/math.pi)
      headsm_running=0;
    elseif byte==string.byte("e") then	
      headangle[2]=headangle[2]-1*math.pi/180;
      headsm_running=0;
     print("Headangle:", headangle[2]*180/math.pi)
    elseif byte==string.byte("c") then	
      headangle[2]=headangle[2]+1*math.pi/180;
      headsm_running=0;
     print("Headangle:", headangle[2]*180/math.pi)

  -- Walk velocity setting
    elseif byte==string.byte("i") then	targetvel[1]=targetvel[1]+0.02;
    elseif byte==string.byte("j") then	targetvel[3]=targetvel[3]+0.1;
    elseif byte==string.byte("k") then	targetvel[1],targetvel[2],targetvel[3]=0,0,0;
    elseif byte==string.byte("l") then	targetvel[3]=targetvel[3]-0.1;
    elseif byte==string.byte(",") then	targetvel[1]=targetvel[1]-0.02;
    elseif byte==string.byte("h") then	targetvel[2]=targetvel[2]+0.02;
    elseif byte==string.byte(";") then	targetvel[2]=targetvel[2]-0.02;

    -- reset OccMap
  elseif byte == string.byte("/") then
    OccupancyMap.reset_map();
  elseif byte == string.byte(".") then
    print("get obstacle");
    OccupancyMap.get_velocity();
--    ocm.set_occ_get_obstacle(1);


   elseif byte==string.byte("-") then
      vcm.set_camera_command(1);
   elseif byte==string.byte("=") then
      vcm.set_camera_command(0);


    --Dive stance settings
   elseif byte==string.byte("t") then
     Motion.event("diveready");
   elseif byte==string.byte("g") then
     dive.set_dive("diveCenter");
     Motion.event("dive");
   elseif byte==string.byte("f") then
     dive.set_dive("diveLeft");
     Motion.event("dive");

  -- HeadFSM setting
    elseif byte==string.byte("`") then
      headsm_running = 1-headsm_running;
      if (headsm_running == 1) then
        HeadFSM.sm:set_state('headSweep');
      end
    elseif byte==string.byte("1") then	
      headsm_running = 1-headsm_running;
      if( headsm_running==1 ) then
--	Speak.talk("Starting head Scan")
        HeadFSM.sm:set_state('headScan');
      end
    elseif byte==string.byte("2") then
      headsm_running = 0; -- Turn off the head state machine
      -- HeadTransform
      local ball = wcm.get_ball();
      local trackZ = Config.vision.ball_diameter; -- Look a little above the ground
      -- TODO: Nao needs to add the camera select
      headangle = vector.zeros(2);
      headangle[1],headangle[2] = HeadTransform.ikineCam(ball.x, ball.y, trackZ);
      print("Head Angles for looking directly at the ball", unpack(headangle*180/math.pi));

    elseif byte==string.byte("3") then	
      kick.set_kick("kickForwardLeft");
      Motion.event("kick");
    elseif byte==string.byte("4") then	
      kick.set_kick("kickForwardRight");
      Motion.event("kick");

   elseif byte==string.byte("5") then	--Turn on body SM
--     Speak.talk("Starting body Search")
     headsm_running=1;
     bodysm_running=1;
     BodyFSM.sm:set_state('bodySearch');   
     HeadFSM.sm:set_state('headScan');
   elseif byte==string.byte("6") then	--Kick head SM
     headsm_running=1;
--     Speak.talk("Starting head Ready")
     HeadFSM.sm:set_state('headReady');
   elseif byte==string.byte("7") then	Motion.event("sit");
   elseif byte==string.byte("8") then	
     if walk.active then walk.stop();end
     Motion.event("standup");
     bodysm_running=0;
   elseif byte==string.byte("9") then	
     Motion.event("walk");
     walk.start();
   elseif byte==string.byte('o') then
     ocm.set_occ_reset(1);
     headangle[2]=50*math.pi/180;
   elseif byte==string.byte('p') then
     -- Change min color for ball
--     if walk.active then walk.stop();end
--     Motion.event('standup')
     headsm_running = 1;
     bodysm_running = 1;
--     HeadFSM.sm:set_state('headLearnLUT');
      vcm.set_camera_learned_new_lut(1)
     HeadFSM.sm:set_state('headLearnLUT');
     BodyFSM.sm:set_state('bodyWait');
--     local ColorLUT = require('ColorLUT')
--     ColorLUT.learn_lut_from_mask();
   end


   -- Apply manul control head angle
   if headsm_running == 0 then
     Body.set_head_command(headangle);
   end

   walk.set_velocity(unpack(targetvel));
 end
end

imageProcessed = false;


function update()
  --Update battery info
  wcm.set_robot_battery_level(Body.get_battery_level());

  local t = Body.get_time();
  Body.set_syncread_enable(0); --read from only head servos
 
  -- Update the Vision
  if t-last_vision_update_time>vision_update_interval then
    last_vision_update_time = t;
    imageProcessed = Vision.update();
  end

  World.update_odometry();
  
  -- Update localization
  if imageProcessed then 
    World.update_vision();
    vcm.refresh_debug_message();
  end

	-- Update Occupancy Map
  if Config.vision.enable_freespace_detection == 1 then
    OccupancyMap.update();
  end
   
  -- Update the relevant engines
  Body.update();
  Motion.update();

  -- Update the HeadFSM if it is running
  if( headsm_running==1 ) then
    HeadFSM.update();
  end

  -- Update the BodyFSM if it is running
  if( bodysm_running==1 ) then
    BodyFSM.update();
  end
  
  -- Get a keypress
  process_keyinput();

  --[[
  attackBearing = wcm.get_attack_bearing();
  vStep = {0.02, 0, 0.2 * attackBearing}

  obs = behaviorObstacle.check_obstacle(vStep)
 if (obs.front) then
    print('obstacle in front found')
  elseif (obs.leftside) then
    print('obstacle on left found')
  elseif (obs.rightside) then
    print('obstacle on right found')
  end  
  
  if obs.left and obs.right then
    freeDir = 1 -- both size occupied, need slow down and backstep
  elseif obs.left then
    freeDir = 2 -- right side free
  elseif obs.right then
    freeDir = 3 -- left side free
  else
    freeDir = 0 -- both size free
  end

  if freeDir == 1 then
    vStep[1] = vStep[1] - 0.01
  elseif freeDir == 2 then
    vStep[3] = vStep[3] - 0.02
  elseif freeDir == 3 then
    vStep[3] = vStep[3] + 0.02
  else
    if angle > 10 * math.pi / 180 then
      vStep = {0, 0, 0.2}
    elseif angle < -10 * math.pi / 180 then
      vStep = {0, 0, -0.2}
    else
      vStep = {0, 0, 0}
    end
  end

  walk.set_velocity(vStep[1], vStep[2], vStep[3])
  --]]
end

while 1 do
  update();
  io.stdout:flush();
end
