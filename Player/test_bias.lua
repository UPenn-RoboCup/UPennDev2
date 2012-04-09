--[[
Servo bias set code for darwin OP by SJ
Haven't checked for nao compatibility at all!!
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
package.path = cwd.."/Vision/?.lua;"..package.path;
package.path = cwd.."/World/?.lua;"..package.path;
package.path = cwd.."/BodyFSM/?.lua;"..package.path;
package.path = cwd.."/HeadFSM/?.lua;"..package.path;

require('Config');
require('Body')
require('vector')
require("getch")
require('Kinematics');

-- initialize state machines
getch.enableblock(1);

jointNames={"HipYaw","HipRoll","HipPitch","KneePitch","AnklePitch","AnkleRoll"};
jointindex=4;
hardness=vector.new({0,0,0,0,0,0});

-- main loop
Body.set_lleg_hardness(0);
Body.set_rleg_hardness(0);
Body.set_syncread_enable(1);
q=vector.zeros(12);
q0=vector.zeros(12);
isended=false;

count=0;
t0 = unix.time();

function init()
  Body.set_larm_hardness(0);
  Body.set_rarm_hardness(0);
  Body.set_head_command({0,0});
end

function info()
  print(" Key commands \n a/d: Left adjust\n w/x: Right adjust\n");
  print(" 1/2: change joint");
  print(" 5: Toggle torque for all joints");
  print(" 6: Toggle torque for current joint pair");
  print(" -: Straighten legs");
  print(" g: Save and Exit")
end

--bias0= Body.get_actuator_bias();
legBias=Config.walk.servoBias;
bias_offset = 5; --Servo id starts with 6
bias = vector.zeros(20);
bias0 = vector.zeros(20);
for i=1,12 do
  bias[i+bias_offset]=legBias[i];
  bias0[i+bias_offset]=legBias[i];
end

hardness_all = 0;

function process_keyinput()
  local str=getch.get();
  if #str>0 then
    local byte=string.byte(str,1);
    q1=vector.slice(Body.get_sensor_position(),6,18);

    if byte==string.byte("5") then 
      hardness_all=1-hardness_all;
      hardness=vector.new({1,1,1,1,1,1})*hardness_all;
    elseif byte==string.byte("6") then 
      hardness[jointindex]=1-hardness[jointindex];	
    elseif byte==string.byte("-") then 
      Body.set_lleg_command(vector.zeros(12));
    elseif byte==string.byte("1") then jointindex=(jointindex-2)%6+1;
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
    elseif byte==string.byte("g") then
      isended=true;
    else info();
    end
    print(string.format("\n %s: L%d, R%d)",
      jointNames[jointindex],
      bias[jointindex+bias_offset],bias[jointindex+6+bias_offset]));
    Body.set_actuator_bias(bias);
    Body.set_lleg_hardness(hardness);
    Body.set_rleg_hardness(hardness);
  end
end

init();
info();
while not isended do
  local tDelay = 0.005 * 1E6; -- Loop every 5ms
  process_keyinput();
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
