cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
require('init')

require('Config');
smindex = 0;

package.path = cwd..'/BodyFSM/'..Config.fsm.body[smindex+1]..'/?.lua;'..package.path;
package.path = cwd..'/HeadFSM/'..Config.fsm.head[smindex+1]..'/?.lua;'..package.path;
package.path = cwd..'/GameFSM/'..Config.fsm.game..'/?.lua;'..package.path;

require('shm')
require('Body')
require('vector')

require('BodyFSM');
require('HeadFSM');
require('Motion');
require('walk');
require('HeadTransform')
require('Speak')
require('vcm')
require('Vision')
require('World')
require('Team')
require('util')
require('wcm')
require('gcm')

darwin = false;
webots = false;

-- Enable OP specific 
if(Config.platform.name == 'OP') then
  darwin = true;
end

-- Enable Webots specific
if (string.find(Config.platform.name,'Webots')) then
  webots = true;
end

-- initialize state machines
HeadFSM.entry();
Motion.entry();
World.entry();
Vision.entry();
Team.entry(); --For receiving ball GPS

Body.set_head_hardness({0.4,0.4});
controller.wb_robot_keyboard_enable(100);

-- main loop
headsm_running=0;
bodysm_running=0;

count = 0;
t0=Body.get_time();
last_update_time=t0;
last_vision_update_time=t0;

headangle=vector.new({0,10*math.pi/180});
targetvel=vector.zeros(3);
vision_update_interval = 0.04; --25fps update

camera_select = 1;

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
   elseif byte==string.byte('p') then
     -- Change min color for ball
--     if walk.active then walk.stop();end
--     Motion.event('standup')
     headsm_running = 1;
     bodysm_running = 1;
     BodyFSM.sm:set_state('bodyWait');
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

  -- Update the relevant engines
  Body.update();
  Motion.update();
  Team.update();

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

end

while 1 do
  update();
  io.stdout:flush();
end

