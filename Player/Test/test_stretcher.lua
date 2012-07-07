cwd = cwd or '.'
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

BodyFSM=require('BodyFSM');
HeadFSM=require('HeadFSM');
require('Motion');
require('walk');
require('Speak')
require('util')
-- For team
Team = require 'TeamPrime'

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

--unix.usleep(1E6*1.0);
if( webots ) then
  controller.wb_robot_keyboard_enable(100);
else
  require('getch')
  getch.enableblock(1);
end

-- initialize state machines
Motion.entry();
--Motion.event("standup");
Team.entry();

Body.set_head_hardness({0.4,0.4});

-- main loop
count = 0;
vcmcount=0;
local t0=Body.get_time();
local last_update_time=t0;
local headangle=vector.new({0,10*math.pi/180});
local sm_running=0;
local last_vision_upfasfdsaasfgate_time=t0;
targetvel=vector.zeros(3);
t_update=2;

--Motion.fall_check=0;
--Motion.fall_check=1;
broadcast_enable=0;
ballcount,visioncount,imagecount=0,0,0;
hires_broadcast=0;

cameraparamcount=1;
broadcast_count=0;
buttontime=0;

--Hack for saffire
Body.set_lleg_command({0,0,0,0,0,0,0,0,0,0,0,0})


function process_keyinput()
  local byte;
  if(webots) then
    byte = controller.wb_robot_keyboard_get_key();
    -- Webots only return captal letter number
    if byte>=65 and byte<=90 then
      byte = byte + 32;
    end
  else
    byte = string.byte(getch.get(),1);
  end
  if byte~=48 then --48 is blank... (at least in webots)
    -- Walk velocity setting
    if byte==string.byte("i") then	targetvel[1]=targetvel[1]+0.02;
    elseif byte==string.byte("j") then	targetvel[3]=targetvel[3]+0.1;
    elseif byte==string.byte("k") then	targetvel[1],targetvel[2],targetvel[3]=0,0,0;
    elseif byte==string.byte("l") then	targetvel[3]=targetvel[3]-0.1;
    elseif byte==string.byte(",") then	targetvel[1]=targetvel[1]-0.02;
    elseif byte==string.byte("h") then	targetvel[2]=targetvel[2]+0.02;
    elseif byte==string.byte(";") then	targetvel[2]=targetvel[2]-0.02;

    elseif byte==string.byte("1") then	
      kick.set_kick("kickForwardLeft");
      Motion.event("kick");
    elseif byte==string.byte("2") then	
      kick.set_kick("kickForwardRight");
      Motion.event("kick");
    elseif byte==string.byte("3") then	
      kick.set_kick("kickSideLeft");
      Motion.event("kick");
    elseif byte==string.byte("4") then	
      kick.set_kick("kickSideRight");
      Motion.event("kick");

    elseif byte==string.byte("5") then
      walk.doWalkKickLeft();
    elseif byte==string.byte("6") then
      walk.doSideKickRight();

    elseif byte==string.byte("7") then	Motion.event("sit");
    elseif byte==string.byte("8") then	
      if walk.active then 
        walk.stopAlign();
      end
      Motion.event("standup");

    elseif byte==string.byte("9") then	
      Motion.event("walk");
      walk.start();

    -- Stretcher specific
    elseif byte==string.byte("s") then -- Search for the stretcher
--      sm_running = 1-sm_running;
      print('entering bodySearch!')
      sm_running = 1;
      BodyFSM.sm:set_state('bodySearch');

    elseif byte==string.byte("p") then -- Execute pickup motion
      sm_running = 0;
--      BodyFSM.sm:set_state('bodyPickup');

    end

    walk.set_velocity(unpack(targetvel));
  end

end

function update()
  Body.set_syncread_enable(0); --read from only head servos

  -- Update the relevant engines
  Body.update();
  Team.update();
  Motion.update();
  if( sm_running==1 ) then
    BodyFSM.update();
--    HeadFSM.update();
  end

  -- Get a keypress
  process_keyinput();
end

local tDelay=0.002*1E6;
local ncount = 100;
local tUpdate = Body.get_time();
while 1 do
  count = count + 1;

  update();

  -- Show FPS
  local t = Body.get_time();
  if(count==ncount) then
    local fps = ncount/(t-tUpdate);
    tUpdate = t;
    count = 1;
    print(fps.." FPS")
  end

  --Wait until dcm has done reading/writing
  --  unix.usleep(tDelay);

end

