--[[
Robot- specific setup code 
Can setup all robot-specific calibration parameters
And automatically appends it to calibration file
--]]


module(... or "", package.seeall)

require('unix')
webots = false;
darwin = true;
local cwd = unix.getcwd();
-- the webots sim is run from the WebotsController dir (not Player)
if string.find(cwd, "WebotsController") then
  webots = true;
  cwd = cwd.."/Player"
  package.path = cwd.."/?.lua;"..package.path;
end

computer = os.getenv('COMPUTER') or "";
--computer = 'Darwin'
if (string.find(computer, "Darwin")) then
   -- MacOS X uses .dylib:
   package.cpath = cwd.."/Lib/?.dylib;"..package.cpath;
else
   package.cpath = cwd.."/Lib/?.so;"..package.cpath;
end

package.path = cwd.."/Util/?.lua;"..package.path;
package.path = cwd.."/Config/?.lua;"..package.path;
package.path = cwd.."/Lib/?.lua;"..package.path;
package.path = cwd.."/Dev/?.lua;"..package.path;
package.path = cwd.."/Motion/?.lua;"..package.path;
package.path = cwd.."/Motion/Walk/?.lua;"..package.path;
package.path = cwd.."/Vision/?.lua;"..package.path;
package.path = cwd.."/World/?.lua;"..package.path;
package.path = cwd.."/BodyFSM/?.lua;"..package.path;
package.path = cwd.."/HeadFSM/?.lua;"..package.path;

require('Config');
require('Body')
require('vector')
require("getch")
require('Kinematics');
require('Motion')

-- initialize state machines

jointNames={"HipYaw","HipRoll","HipPitch","KneePitch","AnklePitch","AnkleRoll"};
jointindex=4;
hardness=vector.new({0,0,0,0,0,0});

if(Config.platform.name == 'OP') then
  darwin = true;
  --SJ: OP specific initialization posing (to prevent twisting)
  Body.set_body_hardness(0.3);
  Body.set_actuator_command(Config.stance.initangle)
end
unix.usleep(1E6*1.0);
Body.set_body_hardness(0);
getch.enableblock(1);

-- main loop
isended=false;

count=0;
t0 = unix.time();
Motion.entry();
motion_running = 1;

function init()
  Body.set_larm_hardness(0);
  Body.set_rarm_hardness(0);
  Body.set_head_command({0,0});
  legBias=Config.walk.servoBias;
  bias_offset = 5; --Servo id starts with 6
  bias = vector.zeros(20);
  bias0 = vector.zeros(20);
  for i=1,12 do
    bias[i+bias_offset]=legBias[i];
    bias0[i+bias_offset]=legBias[i];
  end

--  footXComp0=Config.walk.footXComp;



  hardness_all = 0;
  targetvel=vector.zeros(3);
end

function info()
  print(" Key commands \n a/d: Left adjust\n w/x: Right adjust\n");
  print(" 1/2: change joint");
  print(" 6: Enable biasing mode");
  print(" g: Save and Exit")
end

function process_keyinput()
  local str=getch.get();
  if #str>0 then
    local byte=string.byte(str,1);
    q1=vector.slice(Body.get_sensor_position(),6,18);

    if byte==string.byte("6") then 
      if motion_running>0 then
	motion_running=0;
        hardness=vector.new({1,1,1,1,1,1});
        Body.set_lleg_hardness(hardness);
        Body.set_rleg_hardness(hardness);
        Body.set_lleg_command(vector.zeros(12));
      else
        motion_running=1;
      end
    end
    if motion_running==0 then
      Body.set_syncread_enable(1);
       if byte==string.byte("1") then jointindex=(jointindex-2)%6+1;
      elseif byte==string.byte("2") then jointindex=jointindex%6+1;
      elseif byte==string.byte("a") then 
	bias[jointindex+bias_offset]=bias[jointindex+bias_offset]-1;
      elseif byte==string.byte("d") then 
	bias[jointindex+bias_offset]=bias[jointindex+bias_offset]+1;
      elseif byte==string.byte("w") then 
	bias[jointindex+bias_offset+6]=bias[jointindex+bias_offset+6]-1;
      elseif byte==string.byte("x") then 
	bias[jointindex+bias_offset+6]=bias[jointindex+bias_offset+6]+1;
      elseif byte==string.byte("s") then
	bias[jointindex+bias_offset]=bias0[jointindex+bias_offset];
	bias[jointindex+bias_offset+6]=bias0[jointindex+bias_offset+6];
      else info();
      end
      print(string.format("\n %s: L%d, R%d)",
        jointNames[jointindex],
        bias[jointindex+bias_offset],bias[jointindex+6+bias_offset]));
      Body.set_actuator_bias(bias);
    else
      Body.set_syncread_enable(0);
      --Now motion sm is running, test_walk mode
      -- Walk velocity setting
      if byte==string.byte("i") then targetvel[1]=targetvel[1]+0.02;
      elseif byte==string.byte("j") then targetvel[3]=targetvel[3]+0.1;
      elseif byte==string.byte("k") then targetvel[1],targetvel[2],targetvel[3]=0,0,0;
      elseif byte==string.byte("l") then targetvel[3]=targetvel[3]-0.1;
      elseif byte==string.byte(",") then targetvel[1]=targetvel[1]-0.02;
      elseif byte==string.byte("h") then targetvel[2]=targetvel[2]+0.02;
      elseif byte==string.byte(";") then targetvel[2]=targetvel[2]-0.02;

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
      elseif byte==string.byte("q") then
        walk.doWalkKickLeft();
      elseif byte==string.byte("w") then
        walk.doWalkKickRight();
      elseif byte==string.byte("e") then
        walk.doSideKickLeft();
      elseif byte==string.byte("r") then
        walk.doSideKickRight();
      elseif byte==string.byte("7") then	
        Motion.event("sit");
      elseif byte==string.byte("8") then	
        if walk.active then walk.stop();end
        Motion.event("standup");
      elseif byte==string.byte("9") then	
        Motion.event("walk");
        walk.start();
      end
      walk.set_velocity(unpack(targetvel));
      print("Command velocity:",unpack(walk.velCommand))
    end

    if byte==string.byte("g") then
      isended=true;
    end
  end
end

init();
info();
while not isended do
  local tDelay = 0.005 * 1E6; -- Loop every 5ms
  process_keyinput();
  if motion_running==1 then
    Motion.update();
    Body.update();
  end

  unix.usleep(tDelay);
end
Body.set_lleg_hardness(0);
Body.set_rleg_hardness(0);

--append at the end of current configuration file
outfile=assert(io.open("./Config/calibration.lua","a+"));
data=string.format("\n\-\- Updated date: %s\n" , os.date() );
data=data..string.format("cal[\"%s\"].servoBias={",unix.gethostname());
for i=1,12 do
  data=data..string.format("%d,",bias[i+bias_offset]);
end
data=data.."};\n";
outfile:write(data);
outfile:flush();
outfile:close();
