cwd = cwd or os.getenv('PWD')
package.path = cwd.."/?.lua;"..package.path;
require('init')

require('Config')
require('Body')
require('Speak')
require('Motion')
require('vector')
require 'boxercm'
package.path = cwd..'/BodyFSM/Boxer/?.lua;'..package.path;
require 'bodyBox'
require 'bodyMimicWalk'

-- Initialize Variables
darwin = false;
webots = false;
teamID   = Config.game.teamNumber;
playerID = Config.game.playerID;
print '=====================';
print('Team '..teamID,'Player '..playerID)
print '=====================';
targetvel=vector.zeros(3);
hri_level = 0

-- Enable OP specific 
if(Config.platform.name == 'OP') then
  darwin = true;
  print('On OP!')
  --SJ: OP specific initialization posing (to prevent twisting)
  Body.set_body_hardness(0.3);
  Body.set_actuator_command(Config.sit.initangle);
  unix.usleep(1E6*1.0);
  Body.set_body_hardness(0);
end

-- Enable Webots specific
if (string.find(Config.platform.name,'Webots')) then
  print('On webots!')
  webots = true;
end

-- Key Input
if( webots ) then
  controller.wb_robot_keyboard_enable( 100 );
else
  require 'getch'
  getch.enableblock(1);
end

--This is robot specific 
init = false;
initToggle = true;
calibrating = false;
ready = false;
if( webots or darwin) then
  ready = true;
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

  if byte==0 then
		return false
	end
	
  -- Walk velocity setting
  if byte==string.byte("i") then	targetvel[1]=targetvel[1]+0.02;
  elseif byte==string.byte("j") then	targetvel[3]=targetvel[3]+0.1;
  elseif byte==string.byte("k") then	targetvel[1],targetvel[2],targetvel[3]=0,0,0;
  elseif byte==string.byte("l") then	targetvel[3]=targetvel[3]-0.1;
  elseif byte==string.byte(",") then	targetvel[1]=targetvel[1]-0.02;
  elseif byte==string.byte("h") then	targetvel[2]=targetvel[2]+0.02;
  elseif byte==string.byte(";") then	targetvel[2]=targetvel[2]-0.02;

  -- Stance commands
  elseif byte==string.byte("q") then
    walk.switch_stance(1);
  elseif byte==string.byte("w") then
    walk.switch_stance(0);
  elseif byte==string.byte("e") then
    walk.switch_stance(2);

  -- Punch commands
  elseif byte==string.byte("a") then
    walk.doPunch(1);
  elseif byte==string.byte("s") then
    walk.doPunch(2); 
  elseif byte==string.byte("d") then
    walk.doPunch(3);
	-- Toggle vel commands and body angle
  elseif byte==string.byte("z") then
    hri_vel = (hri_vel+1)%3

  -- Walk commands
  elseif byte==string.byte("7") then
    Motion.event("sit");
  elseif byte==string.byte("8") then
    if walk.active then 
      walk.stopAlign();
    end
    Motion.event("standup");
  elseif byte==string.byte("9") then
    Motion.event("walk");
    walk.start();
  elseif byte==string.byte("0") then
		-- Toggle Body Stabilization
    walk.no_stabilize = not walk.no_stabilize;
  end
	
	--print("Command velocity:",unpack(walk.velCommand))
  walk.set_velocity(unpack(targetvel));
	
	return true
  
end

function update()
  count = count + 1;

  if (not init)  then
    if (calibrating) then
      if (Body.calibrate(count)) then
        Speak.talk('Calibration done');
        calibrating = false;
        ready = true;
      end

    elseif (ready) then
      init = true;
    else
      if (count % 20 == 0) then
        if (Body.get_change_state() == 1) then
          Speak.talk('Calibrating');
          calibrating = true;
        end
      end

      -- toggle state indicator
      if (count % 100 == 0) then
        initToggle = not initToggle;
        if (initToggle) then
          Body.set_indicator_state({1,1,1}); 
        else
          Body.set_indicator_state({0,0,0});
        end
      end
    end

  else
    -- Update State Machines 
    Motion.update();
    Body.update();
		update_boxer()
  end

  local dcount = 50;
  if (count % dcount == 0) then
    --    print('fps: '..(50 / (unix.time() - tUpdate)));
    tUpdate = unix.time();
    -- update battery indicator
    Body.set_indicator_batteryLevel(Body.get_battery_level());
  end

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

function update_boxer()
  if( boxercm.get_body_enabled() ) then
		if hri_vel==0 then
			bodyBox.update()
		elseif hri_vel==1 then
			bodyMimic.update()
		elseif hri_vel==2 then
			-- Make sure the skeleton module uses constraints
			bodyMimic.update()
		end
  else
    walk.upper_body_override_off();
  end
end

-- Initialize
Motion.entry()
count = 0;
lcount = 0;
tUpdate = unix.time();
Motion.event("standup");

-- if using Webots simulator just run update
local tDelay = 0.005 * 1E6; -- Loop every 5ms
while (true) do
	-- Run Updates
  process_keyinput();
  update();

  -- Debug Messages
  t_diff = Body.get_time() - (t_last or 0);
  if(t_diff>1) then
    print('Stabilized: ', not walk.no_stabilize )
		if hri_level==0 then
			print('PrimeSense Controls velocity')
    	print("Command velocity:",unpack(walk.velCommand))
		elseif hri_level==1 then
			print('PrimeSense Controls body angle')
	    print(string.format('RPY: %.1f %.1f %.1f\n',unpack(180/math.pi*bodyMimicWalk.rpy)))
		elseif hri_level==2 then
			print('PrimeSense Controls in Null Space')
		end
    t_last = Body.get_time();
  end

  if(darwin) then
    unix.usleep(tDelay);
  end
end
